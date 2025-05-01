#include "motion.h"

void GPIO_init(void) {
    // Enable clock for GPIOB and GPIOD
    RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB clock
    RCC->AHB1ENR |= (1 << 3);  // Enable GPIOD clock

    // Set PB0 as input (PIR sensor)
    GPIOB->MODER &= ~(0x03 << 0);  // Set PB0 as input (00)
    
    // Set PD15 as output (Blue LED)
    GPIOD->MODER |= (0x01 << 28);  // Set PD15 as output (01)
    GPIOD->OTYPER &= ~(1 << 15);   // Push-pull mode
    GPIOD->OSPEEDR |= (0x03 << 30);  // High speed
    GPIOD->PUPDR &= ~(0x03 << 30);   // No pull-up/pull-down
}

uint8_t motion_detected(void) {
    // Check if PB0 is high, return 1 for motion detected, otherwise return 0
    if (GPIOB->IDR & (1 << 0)) {
        return 1;  // Motion detected (PB0 is high)
    } else {
        return 0;  // No motion detected (PB0 is low)
    }
}


