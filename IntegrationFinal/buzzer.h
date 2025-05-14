#ifndef BUZZER_H
#define BUZZER_H

#include "stdio.h"
#include "stm32f407xx.h"
#include "stdbool.h"

extern volatile int buzz_high;
extern volatile int buzz_low;
extern volatile uint8_t buzzer_state;
extern volatile int buzz_power;


void Buzzer_Init(void);
void Beep_Timer_Init(void);
void TIM2_IRQHandler(void);

#endif
