#include "stm32l552xx.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Macros for LED
#define GREEN_LED_PORT GPIOC
#define GREEN_LED_PIN 7  // PC7

// UART Buffer
#define RX_BUFFER_SIZE 128
static char rx_buffer[RX_BUFFER_SIZE];
static volatile int rx_index = 0;
static volatile bool message_received = false;

// Function prototypes
void initUART(void);
void initLED(void);
void delayMs(uint32_t ms);
void USART2_SendString(const char *str);
void blinkGreenLED(int times, int delay);

// Interrupt handler prototype
void USART2_IRQHandler(void);

// States
typedef enum {
    STATE_IDLE,
    STATE_RETURN
} State;

// Global variables
State current_state = STATE_IDLE;

int main(void) {
    // Initialize peripherals
    initUART();
    initLED();

    while (1) {
        switch (current_state) {
            case STATE_IDLE: {
                printf("State: Idle\n");

                // Wait for a message
                if (message_received) {
                    message_received = false;  // Clear the flag
                    if (strcmp(rx_buffer, "PRINT:COMPLETE") == 0) {
                        printf("Signal received: PRINT:COMPLETE. Sending acknowledgment...\n");
                        USART2_SendString("ACK:COMPLETE\n");
                        current_state = STATE_RETURN;
                    } else {
                        printf("Unexpected signal: %s\n", rx_buffer);
                    }
                    memset(rx_buffer, 0, RX_BUFFER_SIZE);  // Clear the buffer
                }
                break;
            }

            case STATE_RETURN: {
                printf("State: Return\n");
                blinkGreenLED(10, 1000);  // Blink green LED 10 times with 1-second delay
                current_state = STATE_IDLE;  // Transition back to idle state
                break;
            }

            default:
                printf("Unknown state!\n");
                current_state = STATE_IDLE;
                break;
        }
    }

    return 0;
}

// Initialize UART2
void initUART(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;  // Enable USART2 clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;     // Enable GPIOA clock

    // Configure PA2 (TX) and PA3 (RX) as alternate function
    GPIOA->MODER &= ~((0b11 << (2 * 2)) | (0b11 << (3 * 2)));  // Clear mode
    GPIOA->MODER |= ((0b10 << (2 * 2)) | (0b10 << (3 * 2)));   // Set alternate mode
    GPIOA->AFR[0] |= ((0x7 << (2 * 4)) | (0x7 << (3 * 4)));    // AF7 for USART2

    // Configure UART
    USART2->BRR = SystemCoreClock / 115200;  // Set baud rate to 115200
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;  // Enable TX, RX, and RX interrupt
    USART2->CR1 |= USART_CR1_UE;  // Enable UART

    // Enable UART interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1);  // Set priority
}

// Initialize LED on PC7
void initLED(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  // Enable GPIOC clock
    GPIOC->MODER &= ~(0b11 << (GREEN_LED_PIN * 2));  // Clear mode bits
    GPIOC->MODER |= (0b01 << (GREEN_LED_PIN * 2));   // Set PC7 as output
    GPIOC->ODR &= ~(1 << GREEN_LED_PIN);  // Turn off LED initially
}

// Delay function (1ms per loop iteration)
void delayMs(uint32_t ms) {
    SysTick->LOAD = 16000 - 1;  // Reload with number of clocks per millisecond
    SysTick->VAL = 0;           // Clear current value register
    SysTick->CTRL = 0x5;        // Enable SysTick timer

    for (uint32_t i = 0; i < ms; i++) {
        while ((SysTick->CTRL & 0x10000) == 0) {}  // Wait until COUNTFLAG is set
    }

    SysTick->CTRL = 0;  // Disable SysTick timer
}

// USART2 Interrupt Handler
void USART2_IRQHandler(void) {
    // Handle RXNE interrupt (Receive Data Register Not Empty)
    if (USART2->ISR & USART_ISR_RXNE) {
        char received = USART2->RDR;  // Read received data

        // Store received character in buffer
        if (rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = received;
            if (received == '\n') {
                rx_buffer[rx_index - 1] = '\0';  // Null-terminate the string
                message_received = true;
                rx_index = 0;  // Reset index for the next message
            }
        } else {
            // Buffer overflow
            rx_index = 0;  // Reset index
        }
    }
}

// Send a string over UART
void USART2_SendString(const char *str) {
    while (*str) {
        while (!(USART2->ISR & USART_ISR_TXE));  // Wait until TXE is set
        USART2->TDR = *str++;  // Transmit character
    }
}

// Blink green LED
void blinkGreenLED(int times, int delay) {
    for (int i = 0; i < times; i++) {
        GPIOC->ODR |= (1 << GREEN_LED_PIN);  // Turn on LED
        delayMs(delay);
        GPIOC->ODR &= ~(1 << GREEN_LED_PIN);  // Turn off LED
        delayMs(delay);
    }
}

// Retarget printf to UART
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        while (!(USART2->ISR & USART_ISR_TXE));  // Wait until TXE is set
        USART2->TDR = ptr[i];  // Transmit character
    }
    return len;
}
