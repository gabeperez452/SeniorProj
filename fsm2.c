#include "stm32l552xx.h"
#include <stdbool.h>

// Function prototypes for delay and states
void delay_ms(uint32_t ms);
//

void state_idle();
void state_checkHeadPosition();
void state_checkBedPosition();
void state_linearActuatorGoUp();
void state_checkVertScaperLimSwitch();
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
    // Enable clock for GPIOC (PC13 for the button) and GPIOB (PB7 for the LED)
    RCC->AHB2ENR |= (1 << 2);  // Enable clock for GPIOC (bit 2 for PC13)
    RCC->AHB2ENR |= (1 << 1);  // Enable clock for GPIOB (bit 1 for PB7)

    // Configure PB7 as output (for blue LED)
    GPIOB->MODER &= ~(3U << (7 * 2));  // Clear mode bits for PB7
    GPIOB->MODER |= (1U << (7 * 2));   // Set PB7 as output (bit index 14 for output)
    
    // Configure PC13 as input (for the button)
    GPIOC->MODER &= ~(3U << (13 * 2));  // Clear mode bits for PC13 (input mode)

    // Enable pull-up resistor for PC13 (since button is active-low)
    GPIOC->PUPDR &= ~(3U << (13 * 2));  // Clear pull-up/pull-down bits for PC13
    GPIOC->PUPDR |= (1U << (13 * 2));   // Set pull-up resistor for PC13

    // Disable the WKUP2 functionality to ensure PC13 works as a regular GPIO
    PWR->CR3 &= ~PWR_CR3_EWUP2;  // Clear the EWUP2 bit to disable WKUP2

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
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __NOP();  // No operation, just burn time (adjust factor for your clock speed)
    }
}
//

// Clock config and enable
void setClks()
{
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}
//


// State functions
void state_idle() {
    printf("State: IDLE\n");

    // Wait for the button to be pressed (PC13 is low when pressed)
    while (1) {
        if ((GPIOC->IDR & (1 << 13)) == 0) {  // Button pressed
            // Turn on the LED connected to PB7
            GPIOB->ODR |= (1 << 7);  // Set PB7 (turn on the LED)

            // Transition to the next state
            current_state = state_linearActuatorGoUp;

            // Break the loop, keeping the LED on after the button press
            break;
        }

        // Small delay for button debouncing
        delay_ms(50);
    }
}



void state_linearActuatorGoUp() {
    printf("State: Linear Actuator Go Up\n");
    // Simulate actuator movement (placeholder code)
   // delay_ms(2000);  // Simulate actuator movement time
    GPIOB->ODR &= ~(1 << 7);  // Turn off LED (just an example, not necessary)
    
    //Linear actuator code
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
    //

    // Transition to the next state
    current_state = state_checkHeadPosition;
}

void state_checkHeadPosition() {
    printf("State: Check Head Position\n");
    // Placeholder logic to check the head position
    delay_ms(1000);

    current_state = state_checkBedPosition;
}

void state_checkBedPosition() {
    printf("State: Check Bed Position\n");
    // Placeholder logic to check the bed position
    delay_ms(1000);

    current_state = state_checkVertScaperLimSwitch;
}

void state_checkVertScaperLimSwitch() {
    printf("State: Check Vertical Scraper Limit Switch\n");
    // Placeholder logic to check the vertical scraper limit switch
    delay_ms(1000);

    current_state = state_scraperArmGoDown;
}

void state_scraperArmGoDown() {
    printf("State: Scraper Arm Go Down\n");
    // Placeholder logic for scraper arm moving down
    delay_ms(1000);

    current_state = state_checkHorizScraperArmLimSwitch;
}

void state_checkHorizScraperArmLimSwitch() {
    printf("State: Check Horizontal Scraper Arm Limit Switch\n");
    // Placeholder logic to check the horizontal scraper arm limit switch
    delay_ms(1000);

    current_state = state_scraperArmGoSwoosh;
}

void state_scraperArmGoSwoosh() {
    printf("State: Scraper Arm Go Swoosh\n");
    // Placeholder logic for scraper arm swooshing
    delay_ms(1000);

    current_state = state_scraperArmGoUp;
}

void state_scraperArmGoUp() {
    printf("State: Scraper Arm Go Up\n");
    // Placeholder logic for scraper arm moving up
    delay_ms(1000);

    current_state = state_idle;  // Return to idle after this
}

void state_error() {
    printf("State: ERROR\n");

    // Error handling, log the issue, turn on error indicator, etc.
    while (1) {
        // Stay in error state (or reset the system, flash an LED, etc.)
        GPIOB->ODR ^= (1 << 7);  // Toggle PB7 as an error indicator
        delay_ms(500);
    }
}
