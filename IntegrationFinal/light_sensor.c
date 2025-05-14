#include "light_sensor.h"
#include "lcd.h"

// Global variable to track the light pulse status
uint8_t pul = 0;  // 0 - no pulse, 1 - pulse sent

// ADC configuration and initialization
void ADC_Config(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= (1U << 8);  // ADC1EN

    // Enable GPIOB and GPIOD clocks
    RCC->AHB1ENR |= (1U << 1);  // GPIOBEN
    RCC->AHB1ENR |= (1U << 3);  // GPIODEN

    // Configure PB0 as analog input (ADC1_IN8)
    GPIOB->MODER |= (3U << 0);  // Set bits [1:0] to '11' for analog mode (PB0)

    // Configure PD15 as output for the blue LED
    GPIOD->MODER |= (1U << 30);    // Output mode
    GPIOD->OTYPER &= ~(1U << 15);  // Push-pull
    GPIOD->OSPEEDR |= (3U << 30);  // High speed
    GPIOD->PUPDR &= ~(3U << 30);   // No pull-up/down

    // Configure ADC1: enable, select channel 8 (PB0)
    ADC1->CR2 |= (1U << 0);         // Enable ADC
    ADC1->SQR3 &= ~(0x1F);          // Clear old channel
    ADC1->SQR3 |= (8U << 0);        // Set channel 8

    // Start ADC calibration
    ADC1->CR2 |= (1U << 2);         // Start calibration
    while (ADC1->CR2 & (1U << 2));  // Wait for calibration to complete
}

// Read ADC value
uint16_t ADC_Read(void) {
    ADC1->CR2 |= (1U << 30);               // Start conversion
    while (!(ADC1->SR & (1U << 1)));       // Wait for EOC
    return ADC1->DR;
}

// Initialize the light sensor
void LightSensor_Init(void) {
    ADC_Config();
}

// Control the blue LED based on light level
void LED_Control(uint16_t light_level) {
    if (light_level < (1U << 11)) {
        GPIOD->ODR |= (1U << 15);  // Turn ON PD15
    } else {
        GPIOD->ODR &= ~(1U << 15); // Turn OFF PD15
    }
}

// Update pulse tracking based on LED state
void light_pulse(void) {
    pul = (GPIOD->ODR & (1U << 15)) ? 1 : 0;
}

// Sleep/armed display logic (optional)
void LCD_sleep(void) {
    static uint8_t last_pulse = 0;

    if (pul != last_pulse) {
        last_pulse = pul;
        LCD_clearDisplay();
        if (pul == 1) {
            LCD_placeCursor(1); delay();
            LCD_printString("     ARMED!     ");
            LCD_placeCursor(2); delay();
            LCD_printString("Detecting Motion");
        } else {
            LCD_placeCursor(1); delay();
            LCD_printString("   Sleep Mode   ");
        }
    }
}

// Handler function
bool light_sensor_handler(void) {
    uint16_t adc_value = ADC_Read();
    LED_Control(adc_value);
    light_pulse();
    return (adc_value < 2000);
}
