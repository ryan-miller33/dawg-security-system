#ifndef MOTION_H
#define MOTION_H

#include "stm32f4xx.h"

void GPIO_init(void);             // Function to initialize GPIO
uint8_t motion_detected(void);    // Function to check if motion is detected

#endif // MOTION_H
