//main.c


#include "stm32f4xx.h"
#include "light_Sensor.h"

int main(void) {
    uint16_t adc_value;

    // Initialize light sensor
    LightSensor_Init();

    while (1) {
        // Read ADC value from the light sensor (photoresistor)
        adc_value = ADC_Read();

        // Control the LED based on the light level
        LED_Control(adc_value);
    }
}
