//accelerometer.h

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "stm32f407xx.h"
#include "stdint.h"

//Accelerometer function prototypes
void SPI_Port_Init(void);
void SPI_Init(void);
uint8_t SPI_Send(uint8_t myData);
void setCS(uint8_t HIGH_LOW);
uint8_t accelerometer_Read_WHOAMI(void);
void accelerometer_Write_Register(uint8_t regAddr, uint8_t data);
void accelerometer_Init(void);
uint8_t accelerometer_Read_Register(uint8_t regAddr);
int16_t accelerometer_Read_Axis(uint8_t lowAddr);

#endif
