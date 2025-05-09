//light_Sensor.h

#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "stm32f4xx.h"

// Global variable to store the pulse status
extern uint8_t light_pulse;


void ADC_Config(void);
uint16_t ADC_Read(void);
void LightSensor_Init(void);
void LED_Control(uint16_t light_level);

#endif



