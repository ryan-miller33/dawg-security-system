// lcd.c
#include "lcd.h"


void LCD_port_init(void){
//STEP 1: Enable GPIOD in RCC AHB1ENR register
	RCC->AHB1ENR |= (1<<3);
//STEP 2: Set MODER of GPIOD Pins 7, 6, 5, 3, 2, 1 & 0 as outputs
	GPIOD->MODER |= (1<<2*RS) | (1<<2*RW) | (1<<2*EN) | (1<<2*DB4) | (1<<2*DB5) | (1<<2*DB6) | (1<<2*DB7);

//STEP 3: Set OTYPER of GPIOD Pins 7, 6, 5, 3, 2, 1 & 0 as push-pull
	
 
//Done with LCD port Initialization
}

/*******************************
 * LCD_init()
 * Inputs: NONE
 * Outputs: NONE
 * LCD Initialization
 * Read the manual carefully
 * We are doing INITIALIZATION BY INSTRUCTION
 * Don't rush it. 
 *******************************
 */

void LCD_init(void){

// STEP 1: Wait for 100ms for power-on-reset to take effect

// OK - nothing needs to be done here. 
// Keep the board powered. By the time your code is downloaded
// to flash and you are ready to start execution using the 
// debugger - 100ms will have passed

// STEP 2: Set RS pin LOW to send instructions
	GPIOD->ODR &= (0<<RS);

// Send instructions using following format:
// Check Busy Falg; Set EN=HIGH; Send 4-bit instruction; Set EN=low;
	GPIOD->ODR |= (1<<EN);
	
	GPIOD->ODR &= ~(1u<<RS)|(1u<<RW)|(1u<<DB7)|(1u<<DB6)|(1u<<DB5)|(1u<<DB4);
	GPIOD->ODR |= (1u<<DB5) | (1u<<DB4); //for first 3 to send
	
	GPIOD->ODR &= ~(1u<<EN);
	
	for (int i=0; i<840000; i++);
	
	GPIOD->ODR |= (1u<<EN);
	
	GPIOD->ODR &= ~(1u<<RS)|(1u<<RW)|(1u<<DB7)|(1u<<DB6)|(1u<<DB5)|(1u<<DB4);
	GPIOD->ODR |= (1u<<DB5)|(1u<<DB4);       // send next 3
	
	GPIOD->ODR &= ~(1u<<EN);
	
	for (int i=0; i<=24000; i++);    
	
	GPIOD->ODR |= (1u<<EN);
	
	GPIOD->ODR &= ~(1u<<RS)|(1u<<RW)|(1u<<DB7)|(1u<<DB6)|(1u<<DB5)|(1u<<DB4);
	GPIOD->ODR |= (1u<<DB5)|(1u<<DB4);       // send next 3
	
	GPIOD->ODR &= ~(1u<<EN);
	
	for (int i=0; i<=24000; i++); 
	
	GPIOD->ODR |= (1u<<EN);
	
	GPIOD->ODR &= ~(1u<<RS)|(1u<<RW)|(1u<<DB7)|(1u<<DB6)|(1u<<DB5)|(1u<<DB4);
	GPIOD->ODR |= (1u<<DB5);       // send next 2
	
	GPIOD->ODR &= ~(1u<<EN);
	
	
	
	
	LCD_sendInstr_NBF(0x28); //contains set and send instructions for initialization but in different form and no check of busy flag

// STEP 4: Set 2 line display -- treats 16 char as 2 lines
//			001DL NF** (DL 0: 4bits; N= 1: 2 lines; F=0 : 5x8 display

// STEP 5: Set DISPLAY to OFF

	LCD_sendInstr(0x08);

// STEP 6: CLEAR DISPLAY

	LCD_sendInstr(0x01);

// STEP 7: SET ENTRY MODE - Auto increment; no scrolling

	LCD_sendInstr(0x06);

// STEP 8: Set Display to ON with Cursor and Blink.

	LCD_sendInstr(0x0F);
	
	LCD_clearDisplay();
	LCD_placeCursor(1);
	
}



/*******************************
 * LCD_placeCursor()
 * Inputs: unsigned integer linenumber
 * Outputs: NONE
 * sets Cursor position to
 * Line 1, character 1 (hex address 0x80)
 * or Line 2, character 1 (hex addres 0xC0)
 *
 *******************************
 */

void LCD_placeCursor(uint32_t lineno){
	
	check_BF(); //check busy flags
	
	if(lineno == 1) {
		
		LCD_sendInstr(0x80); //instruction for placing cursor on line 1
		
	}
	else if (lineno == 2) {
		LCD_sendInstr(0xC0); //for line 2
		
	}
	
}



/*******************************
 * LCD_sendData()
 * Inputs: unsigned character data (8-bit)
 * Outputs: NONE
 * writes the character to LCD.
 * Since we are using 4-bit mode
 * this function will take the character (8-bit)
 * transmit upper 4 bits and then lower 4 bits.
 * make sure the RS, RW and EN signals are set to correct value
 * for each 4-bit. 
 * also make sure to check the BF
 *******************************
 */

void LCD_sendData(unsigned char data)
{
    check_BF(); // Ensure LCD isn't busy

    set_PIN(RS);  // Set RS to indicate data mode
    clear_PIN(RW); // Clear RW to write mode

    // Send upper 4 bits first
    clear_PIN(DB7); //make sure pins are clear each time
    clear_PIN(DB6);
    clear_PIN(DB5);
    clear_PIN(DB4);

    set_PIN(EN);
	
    if (data & (1u<<7)){ //send upper 4 bits
set_PIN(DB7);
}
if (data & (1u<<6)){
set_PIN(DB6);
}
if (data & (1u<<5)){
set_PIN(DB5);
}
if (data & (1u<<4)){
set_PIN(DB4);
}

    clear_PIN(EN); //latch and clear before sending lower bits
    clear_PIN(DB7);
    clear_PIN(DB6);
    clear_PIN(DB5);
    clear_PIN(DB4);

    set_PIN(EN);

    if (data & (1u<<3)){ //send lower 4 bits
set_PIN(DB7);
}
if (data & (1u<<2)){
set_PIN(DB6);
}
if (data & (1u<<1)){
set_PIN(DB5);
}
if (data & (1u<<0)){
set_PIN(DB4);
}
    clear_PIN(EN);

		clear_PIN(DB7); //clear pins
    clear_PIN(DB6);
    clear_PIN(DB5);
    clear_PIN(DB4);

}

void LCD_sendInstr_NBF(unsigned char Instr) //this is an added function that is apart of the init that makes sure setup is correct with a delay instead of a busy flag
{
	
	for(int i=0; i<=840000; i++); //delay for configure
	
	clear_PIN(RS); //same idea as send instruction/data function
	clear_PIN(RW);
	
	clear_PIN(DB7);
	clear_PIN(DB6);
	clear_PIN(DB5);
	clear_PIN(DB4);
	
	set_PIN(EN);
	
	if (Instr & (1u<<7)){
		set_PIN(DB7);
	}
	if (Instr & (1u<<6)){
		set_PIN(DB6);
	}
	if (Instr & (1u<<5)){
		set_PIN(DB5);
	}
	if (Instr & (1u<<4)){
		set_PIN(DB4);
	}

	clear_PIN(EN);
	
	clear_PIN(DB7);
	clear_PIN(DB6);
	clear_PIN(DB5);
	clear_PIN(DB4);

	set_PIN(EN);
	
	if (Instr & (1u<<3)){
		set_PIN(DB7);
	}
	if (Instr & (1u<<2)){
		set_PIN(DB6);
	}
	if (Instr & (1u<<1)){
		set_PIN(DB5);
	}
	if (Instr & (1u<<0)){
		set_PIN(DB4);
	}
	
	clear_PIN(EN);
	
	clear_PIN(DB7);
	clear_PIN(DB6);
	clear_PIN(DB5);
	clear_PIN(DB4);

}

/*******************************
 * LCD_sendInstr()
 * Inputs: unsigned character INSTRUCTION (8-bit)
 * Outputs: NONE
 * Sends commands to LCD
 * We are using 4-bit mode but 
 * this function accepts (8-bit) character
 * as input. You can make the call on how to handle that.
 * make sure the RS, RW and EN signals are set to correct value
 * for each 4-bit part of instructions. 
 * also make sure to check the BF
 *******************************
 */

void LCD_sendInstr(unsigned char Instruction)
{
check_BF();
	
	clear_PIN(RS); //same as above, but basis of instructions to be sent for lcd, but does check busy flag
	clear_PIN(RW);
	
	clear_PIN(DB7);
	clear_PIN(DB6);
	clear_PIN(DB5);
	clear_PIN(DB4);
	
	set_PIN(EN);
	
	if (Instruction & (1u<<7)){
		set_PIN(DB7);
	}
	if (Instruction & (1u<<6)){
		set_PIN(DB6);
	}
	if (Instruction & (1u<<5)){
		set_PIN(DB5);
	}
	if (Instruction & (1u<<4)){
		set_PIN(DB4);
	}

	clear_PIN(EN);
	
	clear_PIN(DB7);
	clear_PIN(DB6);
	clear_PIN(DB5);
	clear_PIN(DB4);

	set_PIN(EN);
	
	if (Instruction & (1u<<3)){
		set_PIN(DB7);
	}
	if (Instruction & (1u<<2)){
		set_PIN(DB6);
	}
	if (Instruction & (1u<<1)){
		set_PIN(DB5);
	}
	if (Instruction & (1u<<0)){
		set_PIN(DB4);
	}
	
	clear_PIN(EN);
	
	clear_PIN(DB7);
	clear_PIN(DB6);
	clear_PIN(DB5);
	clear_PIN(DB4);

}

void LCD_printString(char myString[]) {
    int i = 0;
    while (myString[i] != '\0') { //can be done by itterating over a string until the null terminate is hit
        LCD_sendData(myString[i]); // Print one character
        for (int j = 0; j <= 2000; j++); // Short delay
        i++; // Move to next character
    }
}


/*******************************
 * LCD_clearDisplay()
 * Inputs: NONE
 * Outputs: NONE
 * Function to erase everything and 
 * clear LCD display
 *******************************
 */
void LCD_clearDisplay(void) {
	
	LCD_sendInstr(0x01); //clears display

}



/*******************************
 * clear_PIN()
 * Inputs: an integer PIN NUMBER (e.g. RW, EN)
 * Outputs: NONE
 * CLEARS PIN in GPIOD to 0
 * Read the Reference manual carefully
 * you can use the BSRR register without masks
 * OR you can use the ODR register WITH &~ (AND-NOT) mask 
 * to clear ONE specified pin.
 *******************************
 */
void clear_PIN(int PINNO){
	
	GPIOD->ODR &= ~(1u<<PINNO);
	
	
}

/*******************************
 * set_PIN()
 * Inputs: an integer PIN NUMBER (e.g. RW, EN)
 * Outputs: NONE
 * SETS PIN in GPIOD to 1
 * Read the Reference manual carefully
 * you can use the BSRR register without masks
 * OR you can use the ODR register WITH | (OR) mask 
 * to SET ONE specified pin.
 *******************************
 */
void set_PIN(int PINNO){
	
	GPIOD->ODR |= (1u<<PINNO);

	
}

/*******************************
 * check_BF()
 * Inputs: NONE
 * Outputs: NONE
 * Checks BF flag on DB7 pin of LCD
 * and prevents code from moving ahead
 * if the BF flag is 1 (indicating LCD busy)
 *******************************
 */

void check_BF(void){
	// STEP 1: Clear RS (set RS=0) as reading flag is an instruction
		GPIOD->ODR &= ~(1u<<RS);
	// STEP 2: set Data Pin 7 connected to GPIOD Pin 3 as input 
	// 		   (no pull-up or pull down setup needed here)
		GPIOD->MODER &= ~(3u<<2*DB7);
	// STEP 3: Set RW = 1 to read the BF flag.
		GPIOD->ODR |= (1u<<RW);
	// STEP 4: Set EN = 1
		GPIOD->ODR |= (1u<<EN);

	// STEP 5: Read the BUSY FLAG on Pin 3 of GPIOD.
	//		   Wait here if BUSY and keep reading pin  
	//         until BF becomes 0 indicating NOT BUSY.
		while(GPIOD->ODR & (1u<<DB7));
	
	// STEP 6: CLEAR EN =0
		GPIOD->ODR &= ~(1u<<EN);
	//STEP 7: CLEAR RW =0 
		GPIOD->ODR &= ~(1u<<RW);
	
	//STEP 8: Set Data Pin 7 connected to GPIOD Pin 3 as output
		GPIOD->MODER |= (1<<2*DB7);

	

}

void LCD_printInt(int number) {
	char intb[16];
	sprintf(intb, "%d", number);
	for (int j = 0; j <= 2000; j++);
	LCD_printString(intb);
}

void delay(void) {
	
	for(int j = 0; j < 5000; j++);
	
}
