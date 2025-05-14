#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "stm32f407xx.h"
#include "stdio.h"

void Ultrasonic_init(void);
void Ultrasonic_init_PWM(void);
void Ultrasonic_init_input_capture(void);
void TIM3_IRQHandler(void);
void Proxy_Distance(int int_distance);

extern volatile int last_threat_level;
extern volatile uint8_t threat_level;
extern volatile uint32_t begTime;
extern volatile uint32_t pulse;
extern volatile double echoPulseSec;
extern volatile uint32_t count;
extern volatile double distance;
extern volatile int int_distance;
extern volatile int closest_distance;

#endif
