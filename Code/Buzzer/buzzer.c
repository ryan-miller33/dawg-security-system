#include "buzzer.h"

// Call once at startup:
extern volatile int buzzer_state;
volatile int buzzer_state = 0;

void Buzzer_Init(void) {
    // Enable GPIOA and TIM3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure PA6 as alternate function for TIM3_CH1
    GPIOA->MODER &= ~(3 << (6 * 2));
    GPIOA->MODER |= (2 << (6 * 2)); // AF mode
    GPIOA->AFR[0] |= (2 << (6 * 4)); // AF2 for TIM3

    // Configure TIM3 for 2.4 kHz PWM
    TIM3->PSC = 65; // No prescaler
    TIM3->ARR = 100; // For 2.4 kHz frequency
    TIM3->CCR1 = 1; // 50% duty cycle

    // PWM mode 1 on channel 1
    TIM3->CCMR1 |= (6 << 4);
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Beep_Timer_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 8399; // Prescaler for 10 kHz timer clock
    TIM2->ARR = 1000; // Initial ARR value for 100 ms
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer

    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        if (buzzer_state) {
            TIM3->CCR1 = 0; // OFF
            buzzer_state = 0;
        } else {
            TIM3->CCR1 = TIM3->ARR / 2; // ON
            buzzer_state = 1;
        }
    }
}




