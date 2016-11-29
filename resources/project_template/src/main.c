#define STM32F051

#include <stdint.h>
#include "stm32f0xx.h"
#include "LCD.h"


int main(void) 
{
	lcd_init(); 				
	lcd_command(LCD_CLEAR_DISPLAY);
	lcd_string("Hello World");
	lcd_command(LCD_GOTO_LINE_2); 	
	lcd_string("***********");

	return 0;
}
