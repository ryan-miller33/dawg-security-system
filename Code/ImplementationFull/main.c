#include "stm32f4xx.h"
#include "lcd.h"
#include "light_Sensor.h"
#include "motion.h"
#include "proximitysensor.h"
#include <stdbool.h>

#define PIR_PORT    GPIOB
#define PIR_PIN     0
#define LIGHT_THRESH 2000   // adjust this to your “dark” ADC threshold

int main(void) {
    uint16_t adc_value;
    bool armed     = false;
    bool pir_last  = false;  // for rising-edge detection

    // 1) One-time inits
    LCD_port_init();
    LCD_init();
    LightSensor_Init();

    // configure PB0 as input (PIR)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    PIR_PORT->MODER  &= ~(3u << (PIR_PIN*2));  // PB0 = input
    PIR_PORT->PUPDR  &= ~(3u << (PIR_PIN*2));  // no pull up/down

    // show initial sleep screen
    LCD_clearDisplay();
    LCD_placeCursor(1);
    LCD_printString("Sleep Mode");

    // 2) Light + “arm” loop
    while (1) {
        // — light sensor path —
        adc_value = ADC_Read();
        LED_Control(adc_value);
        light_pulse();
        LCD_sleep();

        // once it goes dark, arm the PIR
        if (!armed && adc_value < LIGHT_THRESH) {
            armed = true;
            LCD_clearDisplay();
            LCD_placeCursor(1);
            LCD_printString("Armed!");
        }

        // only look at PB0 once armed
        if (armed) {
            bool pir_now = (PIR_PORT->IDR & (1u << PIR_PIN)) != 0;

            // rising edge?
            if (pir_now && !pir_last) {
                motion_detec();   // optional user feedback
                break;            // exit to ultrasonic mode
            }
            pir_last = pir_now;
        }
    }

    // 3) Motion seen — fire up the ultrasonic
    Ultrasonic_init();
    Ultrasonic_init_PWM();
    Ultrasonic_init_input_capture();

    LCD_clearDisplay();
    LCD_placeCursor(1);
    LCD_printString("Proximity Mode");

    // 4) Hand off to TIM3 IRQ + Proxy_Distance()
    while (1) {
        __WFI();  // sleep until next interrupt
    }
}
