#include "buzzer.h"
#include "accelerometer.h"

volatile int16_t preX = 0, preY = 0, preZ = 0;

void SPI_Port_Init(void) {
	
	//Enable GPIOA + E | SPI1
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN);
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	//Port E pin 3 as output
	GPIOE->MODER |= (1u<<6);
	
	//GPIOA PINS 5, 6, 7 to AF
	GPIOA->MODER |= (2u<<10 | 2u<<12 | 2u<<14);
	
	//Set GPIOA pins 5, 6, 7 to AF mode register
	GPIOA->AFR[0] |= (5u<<20 | 5u<<24 | 5u<<28);
	
	// Enable GPIOD clock
RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

// Set PD12 (green LED) to output mode
GPIOD->MODER &= ~(3u << (12*2)); // Clear mode bits
GPIOD->MODER |=  (1u << (12*2)); // Set mode to output

}

void SPI_Init(void){
//Generate Serial Clock (SCK) at the right frequency (set BR[2:0]).
//Setting BR to 111, the slowest freq
SPI1->CR1 |= (7u<<3);
//Put SPI into SPI mode 3 CPOL = 1 and CPHA=1
SPI1->CR1 |= (3u<<0);
//Set MSDTR to 1 to make device controller
SPI1->CR1 |= (1u<<2);
//Select Data frame format (DFF)_: 8-bit (default) or 16 bit. Keep default
//Set or clear SSM and SSI bits to configure the Chip Select pin in hardware
SPI1->CR1 |= (1u<<8 | 1u<<9);
//Set SPE to enable SPI
SPI1->CR1 |= (1u<<6);
}

uint8_t SPI_Send(uint8_t myData){
	// Wait until TXE (Transmit buffer empty) flag is set
    while(!(SPI1->SR & SPI_SR_TXE));
    
    // Write data to the SPI data register
    SPI1->DR = myData;
    
    // Wait until RXNE (Receive buffer not empty) flag is set
    while(!(SPI1->SR & SPI_SR_RXNE));
    
    // Read the data from the SPI data register
    return SPI1->DR;
// send SPI data on DR
}
void setCS(uint8_t HIGH_LOW){
// function sets CS signal o HIGH (1) or LOW (0)
if(HIGH_LOW==0)
GPIOE->BSRR = GPIO_BSRR_BR3;
else if (HIGH_LOW==1)
GPIOE->BSRR = GPIO_BSRR_BS3;
}

uint8_t accelerometer_Read_WHOAMI(void) {
	setCS(0);
	SPI_Send(0x8F); //send address with read bit
	uint8_t tag = SPI_Send(0x00); //Read back
	setCS(1);
	
	return tag;
	
}

void accelerometer_Write_Register(uint8_t regAddr, uint8_t data) {
    setCS(0);
    SPI_Send(regAddr & 0x3F); // Write mode (bit 7 = 0)
    SPI_Send(data);
    setCS(1);
}

void accelerometer_Init(void) {
    // CTRL_REG4: Enable X, Y, Z and set data rate to 100Hz
    accelerometer_Write_Register(0x20, 0x47);
    
    // CTRL_REG5: Configure full scale 2g (default)
    accelerometer_Write_Register(0x24, 0x00);
	
	// CTRL_REG3 (0x23) set IF_ADD_INC = 1
accelerometer_Write_Register(0x23, (1<<4));

}

uint8_t accelerometer_Read_Register(uint8_t regAddr) {
    setCS(0);
    SPI_Send(regAddr | 0x80); // Read mode (bit 7 = 1)
    uint8_t data = SPI_Send(0x00);
    setCS(1);
    return data;
}

int16_t accelerometer_Read_Axis(uint8_t lowAddr) {
    uint8_t low, high;
    setCS(0);
    SPI_Send(lowAddr | 0x80); // bit7=1 for read
    low  = SPI_Send(0x00);    // pull in OUT_*_L
    high = SPI_Send(0x00);    // then OUT_*_H
    setCS(1);
    return (int16_t)((high << 8) | low);
}

void accelerometer_handler(void) {
				int16_t x = accelerometer_Read_Axis(0x28);
        int16_t y = accelerometer_Read_Axis(0x2A);
        int16_t z = accelerometer_Read_Axis(0x2C);

       // Inside your movement detection block:
if (abs(x - preX) > MOVEMENT_THRESHOLD ||
    abs(y - preY) > MOVEMENT_THRESHOLD ||
    abs(z - preZ) > MOVEMENT_THRESHOLD) {
    
    GPIOD->ODR |= (1u<<12);  // Turn ON Green LED

    // ?? Start the buzzer if not already started
    
    Beep_Timer_Init();
      

    buzz_high = 100; // Fast beeping (adjust to taste)
    buzz_low  = 50;
		buzz_power = buzz_high/2;
			
    TIM2->ARR = buzz_low;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN; // Ensure timer is running

} else {
    GPIOD->ODR &= ~(1u<<12);  // Turn OFF Green LED

    // ?? Optionally stop the buzzer after movement stops          // Clear output
}


        preX = x;
        preY = y;
        preZ = z;
			}
