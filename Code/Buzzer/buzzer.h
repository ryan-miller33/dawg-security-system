#ifndef BUZZER_H
#define BUZZER_H

#include "stdio.h"
#include "stm32f407xx.h"

void Buzzer_Init(void);
void Beep_Timer_Init(void);
void TIM2_IRQHandler(void);

#endif


