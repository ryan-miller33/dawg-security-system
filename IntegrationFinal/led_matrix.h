#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include "stm32f407xx.h"
#include "stdio.h"

// Initialize TIM5 to toggle PD15 at 1 Hz
void TIM5_Init(uint32_t pin);
void LED_On(int pin);
void TIM5_IRQHandler(void);

#endif // LED_MATRIX_H
