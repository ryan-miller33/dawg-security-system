#include "light_Sensor.h"
#include "lcd.h"





// Global variable to track the light pulse status
uint8_t pul = 0;  // 0 - no pulse, 1 - pulse sent

// ADC configuration and initialization
void ADC_Config(void) {
    // Enable ADC1 clock (RCC_APB2ENR_ADC1EN)
    RCC->APB2ENR |= (1U << 8);  // Set the bit 8 (ADC1EN) of RCC_APB2ENR register to 1

    // Enable GPIOA clock (RCC_AHB1ENR_GPIOAEN)
    RCC->AHB1ENR |= (1U << 0);  // Set the bit 0 (GPIOAEN) of RCC_AHB1ENR register to 1

    // Enable GPIOD clock (RCC_AHB1ENR_GPIODEN)
    RCC->AHB1ENR |= (1U << 3);  // Set the bit 3 (GPIODEN) of RCC_AHB1ENR register to 1

    // Configure PA0 as analog input (ADC1_IN0)
    GPIOA->MODER |= (3U << 0);  // Set bits [1:0] of GPIOA_MODER to '11' for analog mode (PA0)

    // Configure PD15 as output for the blue LED
    GPIOD->MODER |= (1U << 30);  // Set bits [31:30] of GPIOD_MODER to '01' for output mode (PD15)
    GPIOD->OTYPER &= ~(1U << 15); // Set bit 15 of GPIOD_OTYPER to '0' for push-pull mode (PD15)
    GPIOD->OSPEEDR |= (3U << 30); // Set bits [31:30] of GPIOD_OSPEEDR to '11' for high speed (PD15)
    GPIOD->PUPDR &= ~(3U << 30);  // Clear bits [31:30] of GPIOD_PUPDR for no pull-up or pull-down resistors (PD15)

    // Configure ADC: Turn on ADC, configure regular channel
    ADC1->CR2 |= (1U << 0);   // Set bit 0 (ADON) in ADC1_CR2 to enable the ADC
    ADC1->SQR3 &= ~(0x1F);    // Clear bits [4:0] of ADC1_SQR3 register to select channel 0 (PA0)
    ADC1->SQR3 |= (0x0 << 0); // Set bits [4:0] of ADC1_SQR3 register to select ADC channel 0 (PA0)

    // Start ADC calibration
    ADC1->CR2 |= (1U << 2);   // Set bit 2 (CAL) in ADC1_CR2 to start the calibration
    while (ADC1->CR2 & (1U << 2)); // Wait until calibration is done
}

// Read ADC value
uint16_t ADC_Read(void) {
    ADC1->CR2 |= (1U << 30);  // Set bit 30 (SWSTART) in ADC1_CR2 to start the conversion
    while (!(ADC1->SR & (1U << 1))); // Wait until the end of conversion (EOC flag)
    return ADC1->DR; // Return the ADC result from the data register
}

// Initialize the light sensor (ADC configuration)
void LightSensor_Init(void) {
    ADC_Config();  // Call ADC configuration
}

// Control the blue LED based on light sensor value and manage the pulse
void LED_Control(uint16_t light_level) {
    // Using bit shifting to check the light level (if below a threshold)
    if (light_level < (1U << 11)) {  // Threshold is 2^11 = 2048
        // Turn on blue LED (PD15) by setting bit 15 of ODR
        GPIOD->ODR |= (1U << 15);  
    } 
		else {
        // Light is on, turn off the blue LED (PD15)
        GPIOD->ODR &= ~(1U << 15); // Turn off blue LED (PD15)
    }
}


void light_pulse(void){
    if (GPIOD->ODR & (1U << 15)) {  // Check if PD15 is high (blue LED is on)
        pul = 1;  // Set pulse to 1 if PD15 is high
    } else {
        pul = 0;  // Reset pulse if PD15 is off
    }



}

void LCD_sleep (void){
				static uint8_t last_pulse = 0;
        // Only update the LCD when the pulse value changes
        if (pul != last_pulse) {  // Check if the pulse state has changed
            last_pulse = pul;  // Update the last_pulse state

            // Clear display and update text based on pulse status
            LCD_clearDisplay();
            if (pul == 1) {
                LCD_placeCursor(1); // Move cursor to line 1
//                wait(840000);        // Wait (you may adjust this delay for visible effect)
                LCD_printString("   ARMED");
                LCD_placeCursor(2); // Move cursor to line 2
//                wait(840000);        // Wait
                LCD_printString("Detecting Motion");
            } else {
                LCD_placeCursor(1); // Move cursor to line 1
//                wait(840000);        // Wait
                LCD_printString(" Sleep Mode");
            }
        }

}
