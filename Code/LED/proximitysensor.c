#include "stm32f407xx.h"
#include "stdio.h"
#include "proximitysensor.h"
#include "led_matrix.h"


extern volatile uint32_t begTime;
volatile uint32_t begTime = 0;

extern volatile uint32_t pulse;
volatile uint32_t pulse = 0;

extern volatile double echoPulseSec;
volatile double echoPulseSec = 0;

extern volatile uint32_t count;
volatile uint32_t count = 0;

extern volatile double distance;
volatile double distance = 0;

extern volatile int int_distance;
volatile int int_distance = 0;

extern volatile int closest_distance;
volatile int closest_distance = 9999;

extern volatile int threat_level;
volatile int threat_level = 0;

extern volatile int last_threat_level;
volatile int last_threat_level = -1;

void Ultrasonic_init(void) {
	
	RCC->AHB1ENR |= (1u<<1); //enable gpiob clk
	RCC->APB1ENR |= (1u<<1); //enable tim3 clk
	
	GPIOB->MODER &= ~(3u<<(TRIG_PWM*2)); //clear bits pin 4
	GPIOB->MODER |= (2u<<(TRIG_PWM*2)); //set mode to alternate function
	
	GPIOB->AFR[0] &= ~(0xFu<<(16)); //clear bit for pin4
	GPIOB->AFR[0] |= (2u<<(16)); //set af2
	
}

void Ultrasonic_init_PWM(void) {
	
	TIM3->PSC = 15; //set auto reload for 35ms with 1HZ
	
	TIM3->ARR = 50000 - 1; 
	
	//set capture compare
	TIM3->CCR1 = 10;
	TIM3->CCMR1 &= ~(7u<<4);
	TIM3->CCMR1 |= (6u<<4);
	
	//enable output compare 
	TIM3->CCMR1 |= (1u<<3);
	TIM3->CCER |= (1u<<0);
	TIM3->CR1 |= (1u<<0);
	
}

void Ultrasonic_init_input_capture(void) {
	
	RCC->AHB1ENR |= (1u<<1); //enable gpiob
	RCC->APB1ENR |= (1u<<1); //enable tim3
	
	GPIOB->MODER &= ~(3u<<(ECHO_PULSE*2)); //clear bits
	GPIOB->MODER |= (2u<<(ECHO_PULSE*2)); //set to alternate function
	
	GPIOB->AFR[0] &= ~(0xFu<<(ECHO_PULSE*4)); //set pb5 for alternate function 2
	GPIOB->AFR[0] |= (2u<<(ECHO_PULSE*4)); //with TIM3 ch2
	
	TIM3->PSC = 15;
	TIM3->ARR = 0xFFFF; //auto reload to max value
	
	TIM3->CCMR1 &= ~(3u<<8); //clear bits
	TIM3->CCMR1 |= (1u<<8); //set 01
	TIM3->CCER &= ~(1u<<5);
	
	TIM3->CCER |= (1u<<4); //capture ch2
	TIM3->DIER |= (1u<<2) | (1u<<0);
	TIM3->CR1 |= (1u<<0); //enable counter
	
	__enable_irq();
	NVIC_SetPriority(TIM3_IRQn, 4);
	NVIC_ClearPendingIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
	
}

void TIM3_IRQHandler(void) {
    
	//timer overflow handler
    if (TIM3->SR & (1u<<0)) //make sure it happens
    {
        count++;
        TIM3->SR &= ~(1u<<0); //uif clear
    }
    if (TIM3->SR & (1u<<2)) {

        uint32_t capture = TIM3->CCR2;   // captured value

        if ((TIM3->CCER & (1u<<5)) == 0)  // rising edge
        {
            count = 0;  //reset for new value in overflow
            begTime = capture;
            TIM3->CCER |= (1u<<5); //change to falling edge detection
        }
        else  //falling edge detection
        {
            uint32_t endTime = capture + (count << 16);

            //echo pulse width
            pulse = endTime - begTime;

            //convert to seconds
            echoPulseSec = pulse / 1000000.0;

            distance = (34300 * echoPulseSec) / 2; //calculate distance
            int_distance = (int)distance;   //float to printable int
            Proxy_Distance(int_distance);// sends the calculated distance to tge speed calculator

            //back to rising edge detection
            TIM3->CCER &= ~(1u << 5);

        }
        // capture flag clear
        TIM3->SR &= ~(1u << 2);
    }
}

void Proxy_Distance(volatile int int_distance) {
    static int last_displayed_distance = -1;

    // Update closest distance if it's a new minimum
    if (int_distance < closest_distance) {
        closest_distance = int_distance;
    }

    // Determine threat level
    if (closest_distance < 10) { //10cm is too close for someone before it sends a max threat alert
        threat_level = 2;
    } else if (closest_distance <= 100) {
        threat_level = 1;
    } else {
        threat_level = 0;
    }

    // Only update screen if threat level has changed
    if (threat_level != last_threat_level) {
        last_threat_level = threat_level;

        LCD_clearDisplay();
        LCD_placeCursor(1);

        if (threat_level == 2) {
						delay();
            LCD_printString("DANGER ALERT");
            LCD_placeCursor(2);
						delay();
            LCD_printString("High Threat!");
						TIM5_Init(5);
					 LED_On(1); 
					LED_On(3); 
        }
        else if (threat_level == 1) {
						delay();
            LCD_printString("Proximity Alert");
	
            // Force update of distance on line 2
            last_displayed_distance = -1;
        }
        else {
						delay();
            LCD_printString("Motion Detected");
						TIM5_Init(1);
						
        }
    }

    // Only update line 2 during LOW THREAT state (level 1)
    if (threat_level == 1 && closest_distance != last_displayed_distance) {
        last_displayed_distance = closest_distance;
        LCD_placeCursor(2);
				delay();
        LCD_printString(" Closest: ");
        LCD_printInt(closest_distance);
        LCD_printString(" cm ");
				TIM5_Init(3);
				LED_On(1); 
    }
}



