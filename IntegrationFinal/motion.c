#include "motion.h"

void GPIO_init(void) {
    // Enable clocks for GPIOA and GPIOD
    RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 3);  // Enable GPIOD clock

    // Set PA0 as input (PIR sensor)
    GPIOA->MODER &= ~(0x03 << 0);  // Set PA0 to input mode (00)

    // Set PD15 as output (Blue LED)
    GPIOD->MODER |= (0x01 << 30);   // Output mode
    GPIOD->OTYPER &= ~(1 << 15);    // Push-pull mode
    GPIOD->OSPEEDR |= (0x03 << 30); // High speed
    GPIOD->PUPDR &= ~(0x03 << 30);  // No pull-up/down
}

uint8_t motion_detected(void) {
    // Check if PA0 is high
    return (GPIOA->IDR & (1 << 0)) ? 1 : 0;
}

void motion_sensor_handler(void) {
    static uint32_t motion_timer = 0;
    static uint8_t active = 0;

    if (motion_detected() && !active) {
        GPIOD->ODR |= (1 << 14);  // Turn on PD14
        motion_timer = 1000;      // Approx. 10 cycles
        active = 1;
    }

    if (active && motion_timer > 0) {
        motion_timer--;
        if (motion_timer == 0) {
            GPIOD->ODR &= ~(1 << 14); // Turn off PD14
            active = 0;
        }
    }
}
