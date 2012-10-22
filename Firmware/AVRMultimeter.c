/*
 *  AVRMultimeter.c
 *
 *  Created: 10/16/2012 
 *  Author: Peter Volgyesi <peter.volgyesi@vanderbilt.edu>
 *  Software: Atmel Studio 6 (avr-gcc)
 *  Hardware: ATTiny26-based LCD Multimeter
 */ 

#define F_CPU (1000000)	// NOTE: Place before including util/delay.h

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "lcd.h"

#define TEST_AREF_LEVEL		2.56
#define TEST_MAX_SAMPLE		1023.0

#define TEST_V_IN	12.2
#define TEST_V_POT	1.8
#define TEST_V_SAMPLE (TEST_MAX_SAMPLE * (TEST_V_IN * (8.2+TEST_V_POT) / (120.2+8.2+TEST_V_POT)) / TEST_AREF_LEVEL)

int main(void)
{
	char buff[16];
	
	lcd_init();
	
	for (;;) {                           /* loop forever */
		lcd_home();
		//ltoa(TEST_V_SAMPLE, buff, 10);
		itoa(2561, buff, 10);
		lcd_print(strrev(buff));	 
		lcd_gotoxy(4, 1);
		lcd_print_P(PSTR("Barnabas"));
		_delay_ms(500);   
    }
}