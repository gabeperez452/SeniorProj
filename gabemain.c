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
void stepEnL();
void stepEnR();
void stepDisL();
void stepDisR();
void stepDirF();
void stepDirB();

void linearUp();
void linearDown();
void linearEn();
void linearDis();

void initInputPins();

uint8_t checkScrapeUp();
uint8_t checkScrapeDown();
uint8_t checkSledBL();
uint8_t checkSledBR();
uint8_t checkSledFL();
uint8_t checkSledFR();

/*
 * Design Pinout
 *
 * OUTPUTS:
 *
 *  BendDir		XX		PE8
 *  BendEn		XX		PE7
 *				NC		GND
 *	StepEnR		XX		PE10
 *	StepPulse	XX		PE12
 *  StepDir		XX		PE14
 *  StepEnL		XX		PE15
 *				NC		PB10
 *  ScrapePWM	XX		PB11
 *
 * INPUTS:
 *
 *  ScrpUp		XX		F13
 *  ScrpDwn		XX		E9
 *
 *  SledBL		XX		E11
 *  SledBR		XX		F14
 *  SledFL		XX		E13
 *  SledFR		XX		F15
 *
 * LPUART1 for UART (hopefully connects to STLINK RxTx, can configure for other pins though)
 *
 * TIM2 CH4 probably for PWM PB11
 */

/* ORDER
 *
 * Wait for button input
 * Auto sets servo to very top
 *
 * Wait for button input
 * check servo pos and set upwards
 * wait for button input
 * pull both sleds forwards
 *
 * wait for button
 * linear up, linear down
 * wait for button
 * sleds back
 * wait for button
 * arm down
 * wait for button
 * sleds forward
 * wait for button
 * arm up
 */

int main (void) {
	setClks();

	initInputPins();
	initOutputPins();
	initServoTIM();

	// en PC13 input
	RCC->AHB2ENR |= (1<<2);
	GPIOC->MODER &= ~(0b11 << (13*2));
	// PUPD ??????? seems to work w/o

	// wait for button input (PC13)
	while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
	delayMs(100);
	while((GPIOC->IDR & (1<<13)) >> 13);

	setServoDeg(0);

	while (1) {

		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);


		// check scraper arm pos, set scrape arm up.
		if (checkScrapeUp()) {
			// if arm is up, start servo at 0
			setServoDeg(0);
		} else if (checkScrapeDown()) {
			// if arm is down, start servo at 90
			setServoDeg(90);
		} else {
			// if arm isn't up or down, start servo at 45 and move up until scrape up is reached
			// if scrape up isn't reached, something is very wrong
			int i = 45;
			setServoDeg(i);
			while ((checkScrapeUp() == 0) & (i > 0) ) {
				setServoDeg(i);
				i--;
				delayMs(100);
			}
			if (i >= 180) {
				// very very bad stop
			}
		}

		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);


		// check sled positions, pull both sleds forwards
		stepDirF();
		stepEnL();
		stepEnR();
		delayMs(200);
		int forwardSteps = 0;
		while (!((checkSledFL()) && (checkSledFR())) && (forwardSteps < 15000) ) {
			// if left sled is front,
			if (checkSledFL()) {
				stepDisL();
			} else {
				stepEnL();
			}

			// if right sled is front
			if (checkSledFR()) {
				stepDisR();
			} else {
				stepEnR();
			}

			// pulse enabled sides
			stepPulse(1000);

		}

		// if steps is over 15000, error
		if (forwardSteps >= 15000) {
			// error
		}




		/* LOOP */

		// wait for start input (button input again)
		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);


		// linear up
		linearUp();
		delayMs(100);
		linearEn();
		delayMs(6000);
		linearDis();
		delayMs(1000);

		// linear down
		linearDown();
		delayMs(100);
		linearEn();
		delayMs(6000);
		linearDis();
		delayMs(1000);

		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);

		// sled completely back, check both sides
		stepDirB();
		stepEnL();
		stepEnR();
		delayMs(200);
		int backwardSteps = 0;
		while (!((checkSledBL()) && (checkSledBR())) && (backwardSteps < 15000) ) {
			// if left sled is back,
			if (checkSledBL()) {
				stepDisL();
			} else {
				stepEnL();
			}

			// if right sled is back
			if (checkSledBR()) {
				stepDisR();
			} else {
				stepEnR();
			}

			// pulse enabled sides
			stepPulse(1000);

		}

		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);


		// drop arm, check down
		// arm is up at 0, down at 90
		int armAngle = 0;
		while ((checkScrapeDown() == 0) && (armAngle < 90)) {
			setServoDeg(armAngle);
			armAngle++;
			delayMs(100);
		}

		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);


		// pull sled forward, check both sides
		stepDirF();
		stepEnL();
		stepEnR();
		delayMs(200);
		forwardSteps = 0;
		while (!((checkSledFL()) && (checkSledFR())) && (forwardSteps < 15000) ) {
			// if left sled is front,
			if (checkSledFL()) {
				stepDisL();
			} else {
				stepEnL();
			}

			// if right sled is front
			if (checkSledFR()) {
				stepDisR();
			} else {
				stepEnR();
			}

			// pulse enabled sides
			stepPulse(600);

		}

		// wait for button input (PC13)
		while(((GPIOC->IDR & (1<<13)) >> 13) == 0);
		delayMs(100);
		while((GPIOC->IDR & (1<<13)) >> 13);


		// arm up, check up
		// arm is up at 0, down at 90
		armAngle = 90;
		while ((checkScrapeUp() == 0) && (armAngle > 0)) {
			setServoDeg(armAngle);
			armAngle--;
			delayMs(100);
		}


		/* END LOOP */

		while (1);




		/*
		if (checkSledFR()) {
			GPIOE->ODR |= (0b1 << 15);
		} else {
			GPIOE->ODR &= ~(0b1 << 15);
		}
		*/

	}

	while(1);

}



uint8_t checkScrapeUp () {
	return ((GPIOF->IDR & (1<<13)) >> 13);
}

uint8_t checkScrapeDown () {
	return ((GPIOE->IDR & (1<<9)) >> 9);
}

uint8_t checkSledBL () {
	return ((GPIOE->IDR & (1<<11)) >> 11);
}

uint8_t checkSledBR () {
	return ((GPIOF->IDR & (1<<14)) >> 14);
}

uint8_t checkSledFL () {
	return ((GPIOE->IDR & (1<<13)) >> 13);
}

uint8_t checkSledFR () {
	return ((GPIOF->IDR & (1<<15)) >> 15);
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

void stepEnL() {
	GPIOE->ODR &= ~(1 << 15);	// EN L
//	delayMs(200);
}

void stepEnR() {
	GPIOE->ODR &= ~(0b1 << 10); // EN R
//	delayMs(200);
}

void stepDisL() {
	GPIOE->ODR |= (1 << 15);	// DISABLE
//	delayMs(200);
}

void stepDisR() {
	GPIOE->ODR |= (1 << 10);	// DISABLE R
}

void stepDirB() {
	GPIOE->ODR |= (1 << 14);	// DIR BACKWARD
	delayMs(1);
}

void stepDirF() {
	GPIOE->ODR &= ~(1 << 14);	// DIR FORWARD
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

void initInputPins() {
/*  ScrpUp		XX		F13
 *  ScrpDwn		XX		E9
 *
 *  SledBL		XX		E11
 *  SledBR		XX		F14
 *  SledFL		XX		E13
 *  SledFR		XX		F15
 */
	// GPIO F 13, 14, 15
	// GPIO E 09, 11
	RCC->AHB2ENR |=  (0b1 << 4);	// GPIOE clk
	RCC->AHB2ENR |=  (0b1 << 5);	// GPIOF clk

	// set input
	GPIOF->MODER &= ~(0b11 << (13*2));
	GPIOF->MODER &= ~(0b11 << (14*2));
	GPIOF->MODER &= ~(0b11 << (15*2));

	GPIOE->MODER &= ~(0b11 << (9*2));
	GPIOE->MODER &= ~(0b11 << (11*2));
	GPIOE->MODER &= ~(0b11 << (13*2));

	// set pull down resistor
	GPIOF->PUPDR &= ~(0b111111 << (13*2));
	GPIOF->PUPDR |=  (0b10 << (13*2));		// F13 pull down
	GPIOF->PUPDR |=  (0b10 << (14*2));		// F14 pull down
	GPIOF->PUPDR |=  (0b10 << (15*2));		// F15 pull down

	GPIOE->PUPDR &= ~(0b11 << (9*2));
	GPIOE->PUPDR |=  (0b10 << (9*2));		// E9 pull down
	GPIOE->PUPDR &= ~(0b11 << (11*2));
	GPIOE->PUPDR |=  (0b10 << (11*2));		// E11 pull down
	GPIOE->PUPDR &= ~(0b11 << (13*2));
	GPIOE->PUPDR |=  (0b10 << (13*2));		// E13 pull down
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

