//Integration Final Main File

#include "ultrasonic.h"
#include "lcd.h"
#include "accelerometer.h"
#include "light_sensor.h"    
#include "motion.h"
#include "buzzer.h"

typedef enum {
	STATE_SLEEP,
	STATE_ARMED,
	STATE_ACTIVE
} SystemState;

SystemState system_state = STATE_SLEEP;
bool motion_triggered = false;

int main (void) {
	
	LCD_port_init();
	LCD_init();
	GPIO_init(); //motion sensor init
	
	
	 SPI_Port_Init();
   setCS(1);
   SPI_Init();
   accelerometer_Init();
	// Init previous reading once
    preX = accelerometer_Read_Axis(0x28);
    preY = accelerometer_Read_Axis(0x2A);
    preZ = accelerometer_Read_Axis(0x2C);
	
	LightSensor_Init();
	
	delay();
	// Show Sleep Mode at startup
	LCD_clearDisplay();
	LCD_placeCursor(1); delay();
	LCD_printString(" System: SLEEP! ");
	LCD_placeCursor(2); delay();
	LCD_printString("Room Lighting On");

	while (1) {
        switch (system_state) {
            case STATE_SLEEP:
                if (light_sensor_handler()) {
                    LCD_clearDisplay();
                    LCD_placeCursor(1); delay();
										LCD_printString(" System: ARMED! ");
										LCD_placeCursor(2); delay();
										LCD_printString(" Scaning Motion ");
                    system_state = STATE_ARMED;
									
                }
                break;

            case STATE_ARMED:
	
						if (light_sensor_handler() == false && !motion_triggered) { //light increases again and no motion goes back to sleep
								check_BF();
								LCD_clearDisplay();
								LCD_placeCursor(1); delay();
								LCD_printString(" System: SLEEP! ");
								LCD_placeCursor(2); delay();
								LCD_printString("Room Lighting On");
							break;
    }
                //light_sensor_handler();  // still needed for LCD
                if (motion_detected()) {
                    LCD_clearDisplay();
                    LCD_placeCursor(1); delay();
                    LCD_printString(" Proximity Mode ");
										LCD_placeCursor(2); delay();
										LCD_printString("    ENABLED!    ");
									
										for(int j = 0; j < 100000; j++);
									
                    Ultrasonic_init();
                    Ultrasonic_init_PWM();
                    Ultrasonic_init_input_capture();

                    system_state = STATE_ACTIVE;
                }
                break;

            case STATE_ACTIVE:
                light_sensor_handler();        // still allowed
                accelerometer_handler();       // for motion
                motion_sensor_handler();       // optional PIR LED
                break;
        }

        delay(); // short delay to debounce/slow cycle
    }
	
}
