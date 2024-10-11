#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void delayMs(int n);
void delay_us(uint32_t val);

void enableServo();
void disableServo();
void setServoDeg(int degrees);

void initServoTIM();
void initOutputPins();
void initInputPins();

void stepPulse(int delay);
void stepEn();
void stepDis();
void stepDirF();
void stepDirB();

void linearUp();
void linearDown();
void linearEn();
void linearDis();


/*
 * Design Pinout
 *
 * OUTPUTS:
 *
 *  BendDir		XX		PE8
 *  BendEn		XX		PE7
 *				NC		GND
 *				NC		PE10
 *	StepPulse	XX		PE12
 *  StepDir		XX		PE14
 *  StepEn		XX		PE15
 *				NC		PB10
 *  ScrapePWM	XX		PB11
 *
 * INPUTS:
 *
 *  ScrpUp		XX
 *  ScrpDwn		XX
 *
 *  SledBL		XX
 *  SledBR		XX
 *  SledFL		XX
 *  SledFR		XX
 *
 *
 *
 * LPUART1 for UART (hopefully connects to STLINK RxTx, can configure for other pins though)
 *
 * TIM2 CH4 probably for PWM PB11
 *
 *
 *
 */


int main (void) {

	// init basics
	setClks();
	LPUART1init();


	/*************
	 * TEST CODE *
	 *************/

	initOutputPins();

	initServoTIM();

	/*
	// linear actuator
	linearUp();
	delayMs(100);
	linearEn();
	delayMs(5000);
	linearDis();
	delayMs(1000);

	linearDown();
	delayMs(100);
	linearEn();
	delayMs(5000);
	linearDis();
	delayMs(1000);
	*/

	// theoretically 75 turns from front to back
	// tested 77 turns
	// one turn = 200 pulses
	// 75 turns = 15,000 pulses
	// 70 turns = 14,000 pulses

	delayMs(2000);

	stepEn();
	stepDirF();
	// step forward front to back
	for (int i = 0; i < 15400; i++) {
		stepPulse(600);
	}

	delayMs(1000);

	stepDirB();
	for (int i = 0; i < 15400; i++) {
		stepPulse(600);
	}


	/*
	delayMs(5000);


	// linear actuator
	linearUp();
	delayMs(100);
	linearEn();
	delayMs(5000);
	linearDis();
	delayMs(1000);

	linearDown();
	delayMs(100);
	linearEn();
	delayMs(5000);
	linearDis();
	delayMs(1000);



	// steppers
	stepEn();
	stepDirF();

	for (int i = 0; i < 1200; i++) {
		stepPulse(600);
	}
	delayMs(100);
	stepDirB();
	for (int i = 0; i < 1200; i++) {
		stepPulse(600);
	}


	// servo
	for (int i = 0; i < 90; i++) {
		setServoDeg(i);
		delayMs(20);
	}
	for (int i = 90; i > 0; i--) {
		setServoDeg(i);
		delayMs(20);
	}
	*/


	while(1) {

	}

}

void setServoDeg(int degrees) {
	// turn off timer
	TIM2->CR1 = 0;

	// change match
	// old range 0 - 180
	// new range 500 - 2500
	int oldRange = 180;
	int newRange = 2000;
	int usPulse = (((degrees) * newRange) / oldRange) + 500;
	TIM2->CCR4 = usPulse - 1;
	// turn on timer
	TIM2->CR1 = 1;

}

void enableServo() {
	TIM2->CR1 = 1;
}

void disableServo() {
	TIM2->CR1 = 0;
}

void initServoTIM() {
	// setup PWM
	// GPIOB clk
	RCC->AHB2ENR |= 1 << 1;

	// set PB11 as TIM2 CH2 af mode
	GPIOB->MODER &= ~(0b11 << 22);
	GPIOB->MODER |=  (0b10 << 22);
	GPIOB->AFR[1] &= ~(0b1111 << 12);
	GPIOB->AFR[1] |=  (0b0001 << 12);

	// setup TIM2 CH4 for us pwm
	RCC->APB1ENR1 |= (1 << 0); 		// clk

	// cycle 	20ms 	(20000us)
	// 0deg		.5ms	(500us)
	// 90deg	1.5ms	(1500us)
	// 180deg	2.5ms	(2500us)
	// range	2ms		(2000us)

	TIM2->PSC = 16 - 1;			// prescale for 1 cycle = 1us
	TIM2->ARR = 20000 - 1;		// reset every 20ms
	TIM2->CCMR2 = 0x6000;		// set switch on matched value
	TIM2->CCR4 = 500 - 1 ;		// set degrees 500-2500
	TIM2->CCER |= (0b1 << 12);	// enable CH4 Compare Mode
	TIM2->CNT = 0;
//	TIM2->CR1 = 1;				// start timer

}

void stepPulse(int delay) {
	GPIOE->ODR |=  (1 << 12);	// pulse pin high
	delay_us(delay);
	GPIOE->ODR &= ~(1 << 12);	// pulse pin low
	delay_us(delay);
}

void stepEn() {
	GPIOE->ODR &= ~(1 << 15);	// EN
	delayMs(200);
}

void stepDis() {
	GPIOE->ODR |= (1 << 15);	// DISABLE
	delayMs(200);
}

void stepDirF() {
	GPIOE->ODR |= (1 << 14);	// DIR forward
	delayMs(1);
}

void stepDirB() {
	GPIOE->ODR &= ~(1 << 14);	// DIR backward
	delayMs(1);
}

void linearUp() {
	GPIOE->ODR |=  (0b1 << 8);	// PE8 up
}

void linearDown() {
	GPIOE->ODR &= ~(0b1 << 8);	// PE8 down
}

void linearEn() {
	GPIOE->ODR |=  (0b1 << 7);	// PE7 up
}

void linearDis() {
	GPIOE->ODR &= ~(0b1 << 7);	// PE7 down
}

void initOutputPins(){
	// GPIOE clks
	RCC->AHB2ENR |= 1<<4;

	// PE 15, 14, 12, 10 as output
	GPIOE->MODER &= ~(0b111100110011 << 20);
	GPIOE->MODER |=  (0b010100010001 << 20);
	// turn all off
	GPIOE->ODR &=   ~(0b000000 << 10);

	// PE 08, 07 output
	GPIOE->MODER &= ~(0b1111 << 14);
	GPIOE->MODER |=  (0b0101 << 14);
	GPIOE->ODR &= ~(0b00 << 7);

}

void initInputPins(){


}

void delay_us(uint32_t val){
    // Using TIM5: A delay function that can delay 1usec to 10 sec

	RCC->APB1ENR1 |= 1<<3;		// enable TIM5 clock
	TIM5->PSC = 16 - 1; 		// set Prescaler to 16, making each cycle 1us
	TIM5->ARR = val - 1;		// set ARR to val
	TIM5->CNT = 0;				// clear current counter value
	TIM5->DIER |= 1;			// enable Update Interrupt Flag
	TIM5->CR1 |= 0b10101;		// enable, count down
	TIM5->SR &= ~1;				// reset the stupid UIF

	while(bitcheck(TIM5->SR, 0) == 0);
}

void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}

void LPUART1init() {

	// enable clocks
//	bitset(RCC->APB1ENR1, 28);  // Enable Clock to PWR Interface
	bitset(RCC->AHB2ENR,   6);  // Enable Clock to GPIOG
//	bitset(RCC->APB1ENR2,  0);  // Enable Clock to LPUART
//	bitset(RCC->CCIPR1,   11);  // Select the high speed internal (HSI) oscillator as the clock to LPUART1 (16MHz)
//	bitclear(RCC->CCIPR1, 10);  //
//	bitset(RCC->CR, 8);         // HSI16 clock enable

	// enable power going to port G
	bitset(PWR->CR2, 9);        // Enable GPIOG power

	// config GPIOG
	//set GPIOG.7 to AF
	bitset(GPIOG->MODER,    15);  // Setting 0b10 in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  // Programming 0b1000
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//set GPIOG.8 to AF
	bitset(GPIOG->MODER,    17);  // Setting 0b10 in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(  GPIOG->AFR[1], 3);  // Programming 0b1000
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

	LPUART1-> PRESC = 0;

	// set baud rate and enable TX and RX LPUART
	// BRR = 256*16000000/115200 =
	LPUART1->BRR = 35555;
	LPUART1->CR1 = 0xD | (1<<5); // Enable Receive Data Not Empty Interrupt (RXNEIE)

}

/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}

void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}

