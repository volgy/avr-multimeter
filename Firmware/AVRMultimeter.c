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


#define LCD_DATA_PORT    PORTB
#define LCD_DATA_DDR     DDRB
#define LCD_DATA0_PIN    3
#define LCD_DATA1_PIN    2
#define LCD_DATA2_PIN    1
#define LCD_DATA3_PIN    0
#define LCD_CONTROL_PORT PORTA
#define LCD_CONTROL_DDR  DDRA
#define LCD_RS_PIN       5
#define LCD_E_PIN        4
#define LCD_LINE1_ADDR	0x0
#define LCD_LINE2_ADDR	0x40

#define LCD_CMD_CLR		0x01
#define LCD_CMD_HOME	0x02
#define LCD_CMD_DEFAULT_MODE	0x06		// Entry Mode: cursors shifts, display does not shift automatically
#define LCD_CMD_DISPLAY_ON  0x0C				// Cursor and blinking are off
#define LCD_CMD_DISPLAY_OFF 0x08
#define LCD_CMD_DDRAM	0x80
#define LCD_CMD_CGRAM	0x40

#define LCD_RST_DATA	(LCD_DATA_PORT &= ~_BV(LCD_DATA0_PIN) & ~_BV(LCD_DATA1_PIN) & ~_BV(LCD_DATA2_PIN) & ~_BV(LCD_DATA3_PIN))


#define AREF_LEVEL		2.56
#define ADC_BITS        10
#define MAX_SAMPLE  	((1UL<<ADC_BITS)-1)
#define AVG_N_SAMPLES   (1UL<<((sizeof(uint16_t)*8) - ADC_BITS))
#define MAX_A_SAMPLE    ((1UL<<(sizeof(uint16_t)*8))-1)

#define V_POT	        1.8 // Nominal setting of the potentiometer
#define V_R1            120.0
#define V_R2            8.2
#define V_A_S_1         ((uint16_t)(MAX_A_SAMPLE * (1.0 * (V_R2+V_POT) / (V_R1+V_R2+V_POT)) / AREF_LEVEL))

#define A_GAIN          20.0
#define A_POT           2.0 // Nominal setting of the potentiometer
#define A_SENSE_R       0.1
#define A_R1            68.0
#define A_R2            18.0
#define A_A_S_1         ((uint16_t)(MAX_A_SAMPLE * (1.0 * A_SENSE_R * (A_R2+A_POT) / (A_R1+A_R2+A_POT)) * A_GAIN / AREF_LEVEL))

void lcd_strobe()
{
	LCD_CONTROL_PORT |=  _BV(LCD_E_PIN);
	_delay_us(10);
	LCD_CONTROL_PORT &= ~_BV(LCD_E_PIN);
}

void lcd_transfer(uint8_t token, uint8_t rs)
{
	LCD_CONTROL_PORT &= ~_BV(LCD_RS_PIN) & ~_BV(LCD_E_PIN);
	if (rs) {
		LCD_CONTROL_PORT |= _BV(LCD_RS_PIN);
	}

	LCD_RST_DATA;
	if (token & 0x80) {
		LCD_DATA_PORT |= _BV(LCD_DATA3_PIN);
	}
	if (token & 0x40) {
		LCD_DATA_PORT |= _BV(LCD_DATA2_PIN);
	}
	if (token & 0x20) {
		LCD_DATA_PORT |= _BV(LCD_DATA1_PIN);
	}
	if (token & 0x10) {
		LCD_DATA_PORT |= _BV(LCD_DATA0_PIN);
	}
	lcd_strobe();
	_delay_ms(10);
	
	LCD_RST_DATA;
	if (token & 0x08) {
		LCD_DATA_PORT |= _BV(LCD_DATA3_PIN);
	}
	if (token & 0x04) {
		LCD_DATA_PORT |= _BV(LCD_DATA2_PIN);
	}
	if (token & 0x02) {
		LCD_DATA_PORT |= _BV(LCD_DATA1_PIN);
	}
	if (token & 0x01) {
		LCD_DATA_PORT |= _BV(LCD_DATA0_PIN);
	}
	lcd_strobe();
	_delay_ms(10);
}

void lcd_data(uint8_t data)
{
	lcd_transfer(data, 1);
}

void lcd_cmd(uint8_t cmd)
{
	lcd_transfer(cmd, 0);
}

void lcd_off()
{
	lcd_cmd(LCD_CMD_DISPLAY_OFF);
}

void lcd_on()
{
	lcd_cmd(LCD_CMD_DISPLAY_ON);
}

void lcd_clr()
{
	lcd_cmd(LCD_CMD_CLR);
}

void lcd_home()
{
	lcd_cmd(LCD_CMD_HOME);
}

void lcd_gotoxy(uint8_t x, uint8_t y)
{
	if (y==0) {
		lcd_cmd(LCD_CMD_DDRAM+LCD_LINE1_ADDR+x);
	}
	else {
		lcd_cmd(LCD_CMD_DDRAM+LCD_LINE2_ADDR+x);
	}
}

void lcd_print(char* str)
{
	while((*str) != '\0') {
		lcd_data(*(str++));
	}
}

void lcd_init()
{
	// Setup all lcd pins as outputs
	LCD_CONTROL_DDR |= _BV(LCD_RS_PIN) | _BV(LCD_E_PIN);
	LCD_DATA_DDR |= _BV(LCD_DATA0_PIN) | _BV(LCD_DATA1_PIN) | _BV(LCD_DATA2_PIN) | _BV(LCD_DATA3_PIN);

	// Wait for the module
	_delay_ms(200);
	
	// To be safe switch back to 8-bit interface for a moment (Function Set)
	LCD_CONTROL_PORT &= ~_BV(LCD_RS_PIN) & ~_BV(LCD_E_PIN);
	LCD_RST_DATA;
	LCD_DATA_PORT |= _BV(LCD_DATA0_PIN) | _BV(LCD_DATA1_PIN);
	lcd_strobe();
	_delay_ms(10);
	lcd_strobe();
	_delay_ms(10);
	lcd_strobe();
	_delay_ms(10);
	
	// Function Set: switch to 4 bit mode (last 8-bit command)
	LCD_RST_DATA;
	LCD_DATA_PORT |= _BV(LCD_DATA1_PIN);
	lcd_strobe();
	_delay_ms(10);
	
	// Using 4 bit mode now. Repeat Function Set command (2 lines, 5x8 chars, 4-bit mode)
	lcd_cmd(0x28);
	
	lcd_off();
	lcd_clr();
	lcd_cmd(LCD_CMD_DEFAULT_MODE);
	lcd_on();
}

void lcd_voltage(uint16_t sample, char* buff)
{
	sample = (sample * 100UL) / (V_A_S_1);
	buff[0] = 'U';
	buff[1] = '=';
	buff[2] = ' ';

	buff[3] = ' ';
	for (int i = 7;  sample || (i>3); i--, sample /= 10) {
		buff[i] = '0' + (sample % 10);
		if (i == 6) {
			buff[--i] = '.';
		}
	}

	buff[8] = ' ';
	buff[9] = 'V';
	buff[10] = '\0';
}

void lcd_ampere(uint16_t sample, char* buff)
{
	uint8_t ma = 0;
	
	if (sample < A_A_S_1) {
		sample = (sample * 10000UL) / (A_A_S_1);
		ma = 1;
	}
	else {
		sample = (sample * 100UL) / (A_A_S_1);
	}

	buff[0] = 'I';
	buff[1] = '=';
	buff[2] = ' ';

	buff[3] = ' ';
	buff[7] = '0';
	for (int i = (ma ? 6 : 7);  sample || (i>3); i--, sample /= 10) {
		buff[i] = '0' + (sample % 10);
		if (i == 6) {
			buff[--i] = '.';
		}
	}

	buff[8] = ma ? 'm' : ' ';
	buff[9] = 'A';
	buff[10] = '\0';
}


int main(void)
{
	char buff[16];
	uint16_t sample = 0;
	
	lcd_init();
	
	for (;;) {                           /* loop forever */
		lcd_home();
		
		lcd_voltage(sample, buff);
		lcd_print(buff);
		
		lcd_gotoxy(0, 1);
		lcd_ampere(sample, buff);
		lcd_print(buff);
		
		sample += 100;
		
		_delay_ms(10);   
    }
}