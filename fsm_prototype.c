#include "stm32l552xx.h"
#include <stdbool.h>

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Function prototypes for delay and states
void delay_ms(uint32_t ms);
//


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
 **/




void state_idle();
void state_checkHeadPosition();
void state_checkBedPosition();
void state_linearActuatorGoUp();
void state_linearActuatorGoDown();
void state_checkVertScraperLimSwitch();
void state_scraperArmGoDown();
void state_checkHorizScraperArmLimSwitch();
void state_scraperArmGoSwoosh();
void state_scraperArmGoUp();
void state_error();

// Global variables
typedef void (*StateFunction)();
StateFunction current_state = state_idle;  // Initial state

typedef enum {
    NO_ERROR,
    CALIBRATION_ERROR
} ErrorCode;

ErrorCode error_code = NO_ERROR;

// Test error (you can simulate error with this)
bool detect_error() {
    return false;  // For now, no error.
}

int main() {

	setClks();
	initOutputPins();
	initInputPins();
	initServoTIM();



    // Run FSM continuously
    while (1) {
        if (detect_error()) {
            current_state = state_error;  // Go to error state if error is detected
        }

        current_state();  // Call the current state function
    }

    return 0;
}

// Delay function (assuming 1 ms per loop iteration, adjust for real timing)
void delayMs(int n)
{
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
//

//
void delay_us(uint32_t val)
{
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
//

// Clock config and enable
void setClks()
{
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN); //Enable GPIOA and GPIOC peripheral clocks
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}
//

//
//**Initialization methods**
//
void initOutputPins()
{
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

	GPIOA->MODER &= ~(GPIO_MODER_MODE9);    // Clear MODER9[1:0] (set to 00 for output mode)
	GPIOA->MODER |= (1 << GPIO_MODER_MODE9_Pos); // Set MODER9[0] to 1 (output mode)
}

void initInputPins()
{
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
}

void initServoTIM()
{
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
//

//
// **CONTROL HELPERS**
//
//**linear actuator**
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
//
//**Servo**
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
//
//

// State functions
void state_idle()
{
	if ((GPIOC->IDR & GPIO_IDR_ID13) == 0)
	{  // PC13 is low (e.g., button pressed)
		//Set PA9 to low (reset)
		GPIOA->BSRR = GPIO_BSRR_BR9;          // Reset PA9 (BR9 in BSRR register)
		current_state = state_idle;
	} else {
	    // 6. Set PA9 to high (set)
	    GPIOA->BSRR = GPIO_BSRR_BS9;          // Set PA9 (BS9 in BSRR register)
		current_state = state_linearActuatorGoUp;
	}
}

void state_linearActuatorGoUp()
{
    printf("State: Linear Actuator Go Up\n");
    // Simulate actuator movement (placeholder code)
    linearUp();
    delayMs(100);
    linearEn();
    delayMs(5000);
    linearDis();
    delayMs(1000);

    current_state = state_linearActuatorGoDown;
}

void state_linearActuatorGoDown()
{
	printf("State: Linear Actuator Go Up\n");
	// Simulate actuator movement (placeholder code)
	linearDown();
	delayMs(100);
	linearEn();
	delayMs(5000);
	linearDis();
	delayMs(1000);

	current_state = state_scraperArmGoDown;
}

//void state_checkHeadPosition() {
//    printf("State: Check Head Position\n");
//    // Placeholder logic to check the head position
//    current_state = state_checkVertCr;
//}
//
//void state_checkBedPosition() {
//    printf("State: Check Bed Position\n");
//    // Placeholder logic to check the bed position
//    delay_ms(1000);
//
//    current_state = state_checkVertScaperLimSwitch;
//}

void state_checkVertScraperLimSwitch() {
    printf("State: Check Vertical Scraper Limit Switch\n");
    // Placeholder logic to check the vertical scraper limit switch
    delay_ms(1000);

    current_state = state_scraperArmGoDown;
}

void state_scraperArmGoDown() {
    printf("State: Scraper Arm Go Down\n");
    for (int i = 0; i < 90; i++) {
    		setServoDeg(i);
    		delayMs(20);
    	}

    current_state = state_scraperArmGoSwoosh;
}

void state_checkHorizScraperArmLimSwitch() {
    printf("State: Check Horizontal Scraper Arm Limit Switch\n");
    // Placeholder logic to check the horizontal scraper arm limit switch
    delay_ms(1000);

    current_state = state_scraperArmGoSwoosh;
}

void state_scraperArmGoSwoosh() {
    printf("State: Scraper Arm Go Swoosh\n");
    stepEn();
    stepDirF();
    // step forward front to back
    for (int i = 0; i < 15000; i++) {
    	stepPulse(600);
    }

    delayMs(1000);

    stepDirB();
    for (int i = 0; i < 15000; i++) {
    	stepPulse(600);
    }

    current_state = state_scraperArmGoUp;
}

void state_scraperArmGoUp() {
    printf("State: Scraper Arm Go Up\n");
    for (int i = 90; i > 0; i--) {
    		setServoDeg(i);
    		delayMs(20);
    	}

    current_state = state_idle;  // Return to idle after this
}

void state_error() {
    printf("State: ERROR\n");

    // Error handling, log the issue, turn on error indicator, etc.
    while (1) {
        // Stay in error state (or reset the system, flash an LED, etc.)
        GPIOB->ODR ^= (1 << 7);  // Toggle PB7 as an error indicator
        delayMs(500);
    }
}
