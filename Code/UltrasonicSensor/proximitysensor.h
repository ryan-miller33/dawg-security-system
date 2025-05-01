#ifndef PROXIMITYSENSOR_H
#define PROXIMITYSENSOR_H

#include "stdio.h"
#include "stm32f407xx.h"

#define TRIG_PWM 4
#define ECHO_PULSE 5

#define RS 7 
#define RW 6 
#define EN 5 
#define DB7 3 
#define DB6 2 
#define DB5 1 
#define DB4 0 

void Ultrasonic_init(void);
void Ultrasonic_init_PWM(void);
void Ultrasonic_init_input_capture(void);
void TIM3_IRQHandler(void);

void Proxy_Distance(int int_distance);

// Function prototypes
void LCD_port_init(void); //complete
void LCD_init(void); //complete
void LCD_placeCursor(uint32_t lineno); //complete
void LCD_sendData(unsigned char data); //complete
void LCD_sendInstr(unsigned char Instruction); //complete
void LCD_clearDisplay(void); //complete
void LCD_sendInstr_NBF(unsigned char Instr);

// Additional functions for various data types
void LCD_printChar(char c);
void LCD_printString(char text[]); //only needed to use this function for data types
void LCD_printInt(int number);
void LCD_printFloat(float number, int decimal_places);

// Helper Functions
void clear_PIN(int PINNO); //complete
void set_PIN(int PINNO); //complete
void check_BF(void); //complete
void delay(void); //complete
              

#endif
