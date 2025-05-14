//light_Sensor.h

#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "stdbool.h"

// Global variable to store the pulse status
extern uint8_t pul;
extern bool motion_triggered;

void ADC_Config(void);
uint16_t ADC_Read(void);
void LightSensor_Init(void);
void LED_Control(uint16_t light_level);
void light_pulse(void);
void LCD_sleep (void);
bool light_sensor_handler (void);

#define LIGHT_THRESH 2000;


#endif

