#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include "stm32f407xx.h"

// Initialize TIM5 to toggle PD15 at 1 Hz
void TIM5_Init(uint32_t pin);
void LED_On(int pin);

#endif // LED_MATRIX_H
