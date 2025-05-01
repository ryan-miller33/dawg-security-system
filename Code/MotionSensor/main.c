#include "stm32f4xx.h"
#include "motion.h"

int main(void) {
    // Initialize GPIO
    GPIO_init();

    // PIR sensor detection loop
    while (1) {
        if (motion_detected()) {
            // Turn on blue LED on PD15
            GPIOD->ODR |= (1 << 14);  // Set PD15 to high

            // Simple 10-second delay using a single for loop
            for (volatile uint32_t i = 0; i < 10000000; i++) {
                // Busy-wait loop, adjusting the number for the required delay
            }

            // Turn off blue LED on PD15
            GPIOD->ODR &= ~(1 << 14);  // Set PD15 to low
        }

        // Continue checking for motion
    }
}
