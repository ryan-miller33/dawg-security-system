#include "buzzer.h"
#include "ultrasonic.h"

// Call once at startup:

volatile int buzz_high = 0;         // ms ON duration
volatile int buzz_low = 0;          // ms OFF duration
volatile uint8_t buzzer_state = 0;    // 0 = off, 1 = on
volatile int buzz_power = 0;          // optional PWM scaling (not always used)

void Buzzer_Init(void) {
    // Enable GPIOA and TIM3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Configure PA6 as alternate function for TIM3_CH1
    GPIOB->MODER &= ~(3u<<(12));
    GPIOB->MODER |= (2u<<(12)); // AF mode
    GPIOB->AFR[0] |= (2u<<(24)); // AF2 for TIM3

    // Configure TIM3 for 2.4 kHz PWM
    TIM4->PSC = 15; // No prescaler
    TIM4->ARR = 416; // For 2.4 kHz frequency
    TIM4->CCR1 = 0; // start at 0

    // PWM mode 1 on channel 1
    TIM4->CCMR1 |= (6u<< 4);
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN; //CEN
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
            TIM4->CCR1 = 0; // OFF
            buzzer_state = 0;
						TIM2->ARR = buzz_low;
        } 
				else {
					if (threat_level == 1) {
						TIM4->CCR1 = buzz_power;
					}
					else {
            TIM4->CCR1 = TIM4->ARR / 2; // ON
					}
            buzzer_state = 1;
						TIM2->ARR = buzz_high;
        }
				TIM2->CNT = 0; //reset counter
    }
}

 