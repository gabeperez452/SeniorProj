#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) & 1      ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

// Helping functions
void setClks();



int main (void) {
	setClks();

	// GPIOE clk
	RCC->AHB2ENR |= 1<<4;

	// PE9 11 input pull down
	GPIOE->MODER &= ~(0b11 << 18);
	GPIOE->MODER &= ~(0b11 << 22);

	GPIOE->PUPDR &= ~(0b11 << 18);
	GPIOE->PUPDR |=  (0b10 << 18);

	GPIOE->PUPDR &= ~(0b11 << 22);
	GPIOE->PUPDR |=  (0b10 << 22);

	// PE15 14 output
	GPIOE->MODER &= ~(0b1111 << 28);
	GPIOE->MODER |=  (0b0101 << 28);




	while (1) {

		if ((GPIOE->IDR & (1<<9)) >> 9) {
			GPIOE->ODR |=  (0b1 << 15);
		} else {
			GPIOE->ODR &= ~(0b1 << 15);
		}

		if ((GPIOE->IDR & (1<<11)) >> 1) {
			GPIOE->ODR |=  (0b1 << 14);
		} else {
			GPIOE->ODR &= ~(0b1 << 14);
		}

	}



}







void setClks(){
	RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
	RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
	RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
	RCC->CCIPR1   &= ~(0x400);
	RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
	RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}

