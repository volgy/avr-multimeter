/*
 *  AVRMultimeter.c
 *
 *  Created: 10/16/2012 
 *  Author: Peter Volgyesi <peter.volgyesi@vanderbilt.edu>
 *  Software: Atmel Studio 6 (avr-gcc)
 *  Hardware: ATTiny26-based LCD Multimeter
 */ 


#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lcd.h"

#define F_CPU (XTAL)
#include <util/delay.h>



/*
** constant definitions
*/
static const PROGMEM unsigned char copyright_glyph[] =
{
	0x07, 0x08, 0x13, 0x14, 0x14, 0x13, 0x08, 0x07,
	0x00, 0x10, 0x08, 0x08, 0x08, 0x08, 0x10, 0x00
};

int main(void)
{
	char buffer[7];
    int  num=134;
    unsigned char i;


    /* initialize display, cursor off */
    lcd_init(LCD_DISP_ON);

    for (;;) {                           /* loop forever */
        /* clear display and home cursor */
        lcd_clrscr();
        
        /* put string to display (line 1) with linefeed */
        lcd_puts("LCD Test Line 1\n");

        /* cursor is now on second line, write second line */
        lcd_puts("Line 2");
        
        /* move cursor to position 8 on line 2 */
        lcd_gotoxy(7,1);  
        
        /* write single char to display */
        lcd_putc(':');
        
        _delay_ms(200);
        
        
        /*
         * Test 2: use lcd_command() to turn on cursor
         */
        
        /* turn on cursor */
        lcd_command(LCD_DISP_ON_CURSOR);

        /* put string */
        lcd_puts( "CurOn");
        
		_delay_ms(200);


        /*
         * Test 3: display shift
         */
        
        lcd_clrscr();     /* clear display home cursor */

        /* put string from program memory to display */
        lcd_puts_P( "Line 1 longer than 14 characters\n" );
        lcd_puts_P( "Line 2 longer than 14 characters" );
        
        /* move BOTH lines one position to the left */
        lcd_command(LCD_MOVE_DISP_LEFT);
        
		_delay_ms(200);

        /* turn off cursor */
        lcd_command(LCD_DISP_ON);
        
        
        /*
         *   Test: Display integer values
         */
        
        lcd_clrscr();   /* clear display home cursor */
        
        /* convert interger into string */
        itoa( num , buffer, 10);
        
        /* put converted string to display */
        lcd_puts(buffer);
        
		_delay_ms(200);
        
        
        /*
         *  Test: Display userdefined characters
         */

       lcd_clrscr();   /* clear display home cursor */
       
       lcd_puts("Copyright: ");
       
       /*
        * load two userdefined characters from program memory
        * into LCD controller CG RAM location 0 and 1
        */
       lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
       for(i=0; i<16; i++)
       {
           lcd_data(pgm_read_byte_near(&copyright_glyph[i]));
       }
       
       /* move cursor to position 0 on line 2 */
       /* Note: this switched back to DD RAM adresses */
       lcd_gotoxy(0,1);
       
       /* display user defined (c), built using two user defined chars */
       lcd_putc(0);
       lcd_putc(1);
       

		_delay_ms(200);
              
    }
}