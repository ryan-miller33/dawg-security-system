#include "led_matrix.h"

// file-scope storage for which pin to toggle
static uint32_t TIM5_togglePin;


// TIM5 interrupt handler: clear update flag and toggle the chosen PDx pin
void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;             // clear update interrupt flag
        GPIOD->ODR ^= (1u << TIM5_togglePin); // toggle PDx
    }
}

void TIM5_Init(uint32_t pin) {
    // 1) remember which pin to toggle
    TIM5_togglePin = pin;

    // 2) Enable clocks for GPIOD and TIM5
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;   // GPIOD clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;    // TIM5 clock

    // 3) Configure PDpin as general-purpose output
    //    (each pin has two MODER bits at position pin*2)
    GPIOD->MODER &= ~(3u << (pin * 2));    // clear mode bits
    GPIOD->MODER |=  (1u << (pin * 2));    // 01 = output

    // 4) Timer base configuration for 1 Hz toggles
    //    APB1 timer clock = 84 MHz ? PSC = 8399 ? 10 kHz timer tick
    //    ARR = 4999 ? overflow every (4999+1)/10000 = 0.5 s ? toggle twice per second
    TIM5->PSC   = 8399;
    TIM5->ARR   = 4999;
    TIM5->DIER |= TIM_DIER_UIE;            // enable update interrupt
    TIM5->CR1  |= TIM_CR1_CEN;             // start timer

    // 5) Enable TIM5 interrupt in NVIC
    NVIC_EnableIRQ(TIM5_IRQn);
}



void LED_On(int pin) {
    // 1) Enable clock for GPIOD
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;  // ensure clock is up

    // 2) Configure PD15 as general-purpose output (01)
    GPIOD->MODER &= ~(3u << (pin * 2));   // clear mode bits for PD15
    GPIOD->MODER |=  (1u << (pin * 2));   // set mode to 01 = output

    // 3) Turn on PD15 by setting the corresponding bit in ODR
    GPIOD->ODR |= (1u << pin);            // drive PD15 high
}



