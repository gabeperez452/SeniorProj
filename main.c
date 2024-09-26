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

void initOutputPins();
void initInputPins();

/*
 * Design Pinout
 *
 * OUTPUTS:
 *
 *  StepDir		XX
 *  StepEn		XX
 *  StepPulse	XX
 *
 *  BendDir		XX
 *  BendEn		XX
 *
 *  ScrapePWM	XX
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
 * TIM2 CH2 probably for PWM PB11
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

	// GPIOE clks
	RCC->AHB2ENR |= 1<<4;

	// PE 15, 14, 12, 10 as output
	GPIOE->MODER &= ~(0b111100110011 << 20);
	GPIOE->MODER |=  (0b010100010001 << 20);
	// turn all off
	GPIOE->ODR &=   ~(0b000000 << 10);


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

	TIM2->PSC = 16000 - 1;		// prescale for 1 cycle = 1ms
	TIM2->ARR = 1000 - 1;		// reset every 1000ms
	TIM2->CCMR2 = 0x6000;		// set switch on matched value
	TIM2->CCR4 = 500 - 1 ;		// switch on half for now
	TIM2->CCER |= (0b1 << 12);	// enable CH4 Compare Mode
	TIM2->CNT = 0;
	TIM2->CR1 = 1;



	while(1) {

		GPIOE->ODR |= (1 << 10);
		delayMs(500);

		GPIOE->ODR &= ~(1 << 10);
		GPIOE->ODR |=  (1 << 12);
		delayMs(500);

		GPIOE->ODR &= ~(1 << 12);
		GPIOE->ODR |=  (1 << 14);
		delayMs(500);

		GPIOE->ODR &= ~(1 << 14);
		GPIOE->ODR |=  (1 << 15);
		delayMs(500);

		GPIOE->ODR &= ~(1 << 15);
		delayMs(500);








	}








}

void initOutputPins(){

	// enable GPIOE clks

}

void initInputPins(){


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

