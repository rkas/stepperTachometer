/*
Stepper Tachometer for ATmega168P & X27.168 (& L293D)

PC0	->	Stepper1
PC1 ->	Stepper2
PC2 ->	Stepper3
PC3 ->	Stepper4
PB0	->	Input(REV)	ICP1
PB1 ->	Input(ILM)
PD0 ->	LED(RED)
PD1 ->	LED(GRN)
PD2 ->	LED(BLU)
PD3 ->	PWM(LED)		OC2B
PD4	->	Buzzer
PD5	->	Button1(Pulled-up)
PD6	->	Button2(Pulled-up)
*/

#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define BLUZONE_RPM 2800
#define GRNZONE_RPM 3400
#define YELZONE_RPM 6800
#define REDZONE_RPM 7400
#define DIMMER_DUTY	32
#define VAL_DIVBY_TP_US 30000000
// RPM = (1 / 2) * ((1 / timePeriod_us) * 1000 * 1000 * 60); //
//          ↑ クランク1回転で2パルス出るため                       //
//     = 30000000 / timePeriod_us                            //

#define NUM_STEPS_ZERO_OFFSET 100	//モーター内蔵ストッパーと文字盤の0点との差(1ステップ=0.333deg)
#define DELAY_STEPS_US 1500	//原点出し時の駆動周期
#define NUM_SAMPLE 5	//パルス幅測定値の移動平均を求めるときに使うサンプル数
#define PLAY_OF_POSITION 0	//targetPositionとcurrentPositionとの許容遊び幅

#define TIMEPERIOD_US_MIN 1832	//これより速い周期は考えない(オーバーフロー対策)
#define NUM_VELOCITIES 48	//配列velocityToOCR0ATableのサイズ。何ステップかけて最低速から最高速まで加減速するか

#define OFF 0
#define ON	1
#define BLU 1
#define GRN 2
#define YEL 3
#define RED 4


// Global variable declaration //

volatile int currentState = 0;	//0-6
volatile int currentPosition = 0;	//0-945 315/(60/180)
volatile int targetPosition = 0;	//0-945
const int pulseStateTable[6] = {
	//3210
	0b0110,
	0b0100,
	0b0001,
	0b1001,
	0b1000,
	0b0010,
};

volatile int velocity = 0;	// -(NUM_VELOCITIES-1)~0~(NUM_VELOCITIES-1)
const unsigned char velocityToOCR0ATable[NUM_VELOCITIES] = {	//Unit: 1 * 64 [us]
	40,  39,  38,  38,  37,  36,  36,  35,
	34,  34,  33,  32,  32,  31,  31,  30,
	29,  29,  28,  27,  27,  26,  25,  25,
	24,  24,  23,  22,  22,  21,  20,  20,
	19,  18,  18,  17,  17,  16,  15,  15,
	14,  13,  13,  12,  11,  11,  10,  10
};

volatile unsigned int countTIMER1Overflow = 0;	//TIMER1のオーバーフロー回数
volatile unsigned long timePeriod_us = 1000000000;	//入力パルスの周期

unsigned char ledBrightness = 32;	//


// Proto-type declaration //

void portInit();
void timerInit();
int myAbs(int x);
void stepUp();
void stepDown();
void zeroCariblate();
int getPositionFromRpm(unsigned int rpm);
void led(int color);
void beep(int state);
int isIlmOn();
void openingCelemony();


// Interrupt service routine //

ISR(TIMER0_COMPA_vect){
	if (targetPosition > (945 - NUM_STEPS_ZERO_OFFSET)) {targetPosition = (945 - NUM_STEPS_ZERO_OFFSET);}
	//if (targetPosition < 0) {targetPosition = 0;}
	if ((myAbs(targetPosition - currentPosition)) < (myAbs(velocity))) {	//currentPositionがtargetPositionの近くだったら
		if (velocity < 0) {	//減速(velocityを0に近づける)
			velocity++;
		} else if (velocity > 0) {
			velocity--;
		}
	} else {	//targetPositionが十分遠ければ
		if ((currentPosition < targetPosition) && (velocity < (NUM_VELOCITIES - 1))) {	//CWに加速(velocityの絶対値を増加)
			velocity++;
		} else if ((currentPosition > targetPosition) && (velocity > -(NUM_VELOCITIES - 1))) {	//CCWに加速
			velocity--;
		}
	}
	if ((myAbs(currentPosition - targetPosition)) > PLAY_OF_POSITION) {
		if (velocity > 0) {
			stepUp();
		} else if (velocity < 0) {
			stepDown();
		}
	}
	OCR0A = velocityToOCR0ATable[myAbs(velocity)];	//次に自身が呼び出されるまでの時間を設定
}

ISR(TIMER1_CAPT_vect){	//パルス立ち下がり時
	TCNT1 = 0;
	timePeriod_us = countTIMER1Overflow * 65536 + ICR1;
	countTIMER1Overflow = 0;
}

ISR(TIMER1_OVF_vect){
	countTIMER1Overflow++;
}


int main(void){
	cli();
	portInit();
	timerInit();
	zeroCariblate();
	sei();

	_delay_ms(100);	//wait for pulse coming
	if (timePeriod_us == 1000000000) {	//no pulse
		openingCelemony();
	}

	unsigned long timePeriod_us_history[NUM_SAMPLE];
	for (int i = 0; i < NUM_SAMPLE; i++) {timePeriod_us_history[i] = 1000000000;}
	unsigned long timePeriod_us_sum;
	unsigned long timePeriod_us_mean;
	unsigned int rpm = 0;
	int j = 0;
	while (1) {
		// パルス周期の移動平均をとる (最大値と最小値は除いて計算) //
		if (timePeriod_us < TIMEPERIOD_US_MIN) {timePeriod_us = TIMEPERIOD_US_MIN;}
		timePeriod_us_history[j] = timePeriod_us;
		j++;	if (j == NUM_SAMPLE) {j = 0;}
		timePeriod_us_sum = 0;
		unsigned long min, max;
		min = max = timePeriod_us_history[j];
		for (int i = 0; i < NUM_SAMPLE; i++) {
			timePeriod_us_sum += (timePeriod_us_history[i]);
			if (timePeriod_us_history[i] < min) {min = timePeriod_us_history[i];}
			if (timePeriod_us_history[i] > max) {max = timePeriod_us_history[i];}
		}
		timePeriod_us_mean = (timePeriod_us_sum - min - max) / (NUM_SAMPLE - 2);
		// パルス周期の平均値から回転数を算出 //
		rpm = VAL_DIVBY_TP_US / timePeriod_us_mean;

		if (countTIMER1Overflow > 4) {rpm = 0;}	//114rpm以下(0.26秒以上無入力)で0rpm扱い
		targetPosition = getPositionFromRpm(rpm);

		if (rpm > REDZONE_RPM) {
			led(RED);
		} else if (rpm > YELZONE_RPM) {
			led(YEL);
		} else if (rpm > GRNZONE_RPM) {
			led(GRN);
		} else if (rpm > BLUZONE_RPM) {
			led(BLU);
		} else {
			led(OFF);
		}

		if (isIlmOn()) {
			OCR2B = DIMMER_DUTY;
		} else {
			OCR2B = 255;
		}
	}
}


// Funcion definidion //

void portInit(){
	DDRC  = 0b00001111;	//Stepper1~4
	DDRB  = 0b00000000;	//Input(REV, ILM)
	DDRD  = 0b00011111;	//LED(R,G,B), PWM, Buzz, Btn1~2
	PORTC = 0b11110000;
	PORTB = 0b11111100;
	PORTD = 0b11100000;	//Btn1~2 pull-up
}

void timerInit(){
	// TIMER0 //	(For Stepper Driving)
	//タイマ／カウンタ0制御レジスタA
	//         ++------ COM1A1:COM1A0
	//         ||++---- COM1B1:COM1B0
	//         ||||  ++ WGM01:WGM00     波形生成種別(3bitの下位2bit)
	TCCR0A = 0b00000010;

	//タイマ／カウンタ0制御レジスタB
	//         +------- FOC0A						0?
	//         |+------ FOC0B						0?
	//         ||  +--- WGM02						波形生成種別(3bitの上位1bit)
	//         ||  |+++ CS02:CS01:CS00	分周
	TCCR0B = 0b00000011;

	//StepperDrivingSpeed (Smaller is higher speed)
	OCR0A = velocityToOCR0ATable[0];
	// 1MHz/(64*(1+OCR0A)) [Hz]
	// 1/(1MHz/(64*(1+OCR0A))) [sec]
	// must be 8, 10, or more?

	//タイマ／カウンタ0割り込みマスクレジスタ
	//              +-- OCIE0B
	//              |+- OCIE0A
	//              ||+ TOIE0
	TIMSK0 = 0b00000010;


	// TIMER1 //	(For Rev Pulse Measuring)
	//タイマ／カウンタ1制御レジスタA
	//         ++------ COM1A1:COM1A0
	//         ||++---- COM1B1:COM1B0
	//         ||||  ++ WGM11:WGM10     波形生成種別(4bitの下位2bit)
	TCCR1A = 0b00000000;

	//タイマ／カウンタ1制御レジスタB
	//         +------- ICNC1
	//         |+------ ICES1
	//         || ++--- WGM13:WGM12     波形生成種別(4bitの上位2bit)
	//         || ||+++ CS12:CS11:CS10  分周
	TCCR1B = 0b00000001;

	//タイマ／カウンタ1割り込みマスクレジスタ
	//           +----- ICIE1
	//           |  +-- OCIE1B
	//           |  |+- OCIE1A
	//           |  ||+ TOIE1
	TIMSK1 = 0b00100001;


	// TIMER2 //	(For LED PWM)
	//タイマ／カウンタ2制御レジスタA
	//         ++------ COM2A1:COM2A0
	//         ||++---- COM2B1:COM2B0
	//         ||||  ++ WGM21:WGM20     波形生成種別(3bitの下位2bit)
	TCCR2A = 0b00110011;

	//タイマ／カウンタ0制御レジスタB
	//         +------- FOC2A						0?
	//         |+------ FOC2B						0?
	//         ||  +--- WGM22						波形生成種別(3bitの上位1bit)
	//         ||  |+++ CS22:CS21:CS20	分周
	TCCR2B = 0b00000001;

	//LED PWM-Duty
	OCR2B = 255;
	// Duty[%] = OCR2B / 255 * 100

	//タイマ／カウンタ2割り込みマスクレジスタ
	//              +-- OCIE2B
	//              |+- OCIE2A
	//              ||+ TOIE2
	TIMSK2 = 0b00000000;
}

int myAbs(int x){
	if (x < 0) {
		return -x;
	} else {
		return x;
	}
}

/* 時計方向に1ステップ進める */
void stepUp(){
	currentPosition++;
	currentState = currentPosition % 6;
	PORTC = pulseStateTable[currentState];
}

/* 反時計方向に1ステップ戻す */
void stepDown(){
	currentPosition--;
	currentState = currentPosition % 6;
	PORTC = pulseStateTable[currentState];
}

void zeroCariblate(){
	cli();
	currentPosition = 1000;
	for (int i = 0; i < 1000; i++) {
		stepDown();
		_delay_us(DELAY_STEPS_US);
	}
	for (int i = 0; i < NUM_STEPS_ZERO_OFFSET; i++) {
		stepUp();
		_delay_us(DELAY_STEPS_US);
	}
	currentPosition = 0;
}

int getPositionFromRpm(unsigned int rpm){
	return (rpm / 10);
}

void led(int color){
	switch (color) {
		case 1:	//BLU
			PORTD &= 0b11111100;
			PORTD |= 0b00000100;
			break;
		case 2:	//GRN
			PORTD &= 0b11111010;
			PORTD |= 0b00000010;
			break;
		case 3:	//YEL
			PORTD &= 0b11111011;
			PORTD |= 0b00000011;
			break;
		case 4:	//RED
			PORTD &= 0b11111001;
			PORTD |= 0b00000001;
			break;
		default:	//OFF
			PORTD &= 0b11111000;
	}
}

void beep(int state){
	if (state) {
		PORTD |= 0b00010000;
	} else {
		PORTD &= 0b11101111;
	}
}

int isIlmOn(){
	return (PINB & 0b00000010);
}

void openingCelemony(){
	const int zoneRpmTable[5] = {0, BLUZONE_RPM, GRNZONE_RPM, YELZONE_RPM, REDZONE_RPM};
	for (int i = 1; i <=4; i++) {
		targetPosition = getPositionFromRpm(zoneRpmTable[i]);
		while (currentPosition != targetPosition);
		led(i);
		_delay_ms(250);
		led(OFF);
	}
	targetPosition = 0;
	while (currentPosition != targetPosition);
}
