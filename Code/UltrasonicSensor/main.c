#include "proximitysensor.h"

int main (void) { //main method
	
	LCD_port_init(); //init gpio for lcd
	LCD_init(); //init for lcd itself contains functions for printing
	//Motor_init(); //dc motor
	//RPG_init(); //rpg init
	LCD_clearDisplay();
	LCD_placeCursor(1);
	LCD_printString(" Motion Detected");
	for(int i=0; i<50000; i++);
//	LCD_clearDisplay();
	Ultrasonic_init();
	Ultrasonic_init_PWM();
	Ultrasonic_init_input_capture();
	
	
	
	
	while (1);

	
}
