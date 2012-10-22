/*
 *  lcd.h	- HD44780 LCD driver for AVRMultimeter
 *
 *  NOTE: This is not a generic driver
 *  NOTE: All functions are inlined (no .c file) for saving stack space
 *
 *  Created: 10/20/2012 
 *  Author: Peter Volgyesi <peter.volgyesi@vanderbilt.edu>
 *  Software: Atmel Studio 6 (avr-gcc)
 *  Hardware: ATTiny26-based LCD Multimeter
 */ 
#ifndef LCD_H
#define LCD_H

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
#define LCD_CLR_DATA	(LCD_DATA_PORT &= ~_BV(LCD_DATA0_PIN) & ~_BV(LCD_DATA1_PIN) & ~_BV(LCD_DATA2_PIN) & ~_BV(LCD_DATA3_PIN))
#define LCD_LINE1_ADDR	0x0
#define LCD_LINE2_ADDR	0x40

#define LCD_CMD_CLR		0x01
#define LCD_CMD_HOME	0x02
#define LCD_CMD_DEFAULT_MODE	0x06		// Entry Mode: cursors shifts, display does not shift automatically
#define LCD_CMD_DISPLAY_ON  0x0C				// Cursor and blinking are off
#define LCD_CMD_DISPLAY_OFF 0x08				
#define LCD_CMD_DDRAM	0x80
#define LCD_CMD_CGRAM	0x40

static const PROGMEM unsigned char copyright_glyph[] =
{
	0x07, 0x08, 0x13, 0x14, 0x14, 0x13, 0x08, 0x07,
	0x00, 0x10, 0x08, 0x08, 0x08, 0x08, 0x10, 0x00
};

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

	LCD_CLR_DATA;
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
	
	LCD_CLR_DATA;
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

void lcd_init()
{
	// Setup all lcd pins as outputs
	LCD_CONTROL_DDR |= _BV(LCD_RS_PIN) | _BV(LCD_E_PIN);
	LCD_DATA_DDR |= _BV(LCD_DATA0_PIN) | _BV(LCD_DATA1_PIN) | _BV(LCD_DATA2_PIN) | _BV(LCD_DATA3_PIN);

	// Wait for the module
	_delay_ms(200);
	
	// To be safe switch back to 8-bit interface for a moment (Function Set)
	LCD_CONTROL_PORT &= ~_BV(LCD_RS_PIN) & ~_BV(LCD_E_PIN);
	LCD_CLR_DATA;
	LCD_DATA_PORT |= _BV(LCD_DATA0_PIN) | _BV(LCD_DATA1_PIN);
	lcd_strobe();
	_delay_ms(10);
	lcd_strobe();
	_delay_ms(10);
	lcd_strobe();
	_delay_ms(10);
	
	// Function Set: switch to 4 bit mode (last 8-bit command)
	LCD_CLR_DATA;
	LCD_DATA_PORT |= _BV(LCD_DATA1_PIN);
	lcd_strobe();
	_delay_ms(10);
	
	// Using 4 bit mode now. Repeat Function Set command (2 lines, 5x8 chars, 4-bit mode)
	lcd_cmd(0x28);
	
	// Display, cursor and blinking are off
	lcd_off();
	
	// Clear Display
	lcd_clr();
	
	// Default Entry Mode
	lcd_cmd(LCD_CMD_DEFAULT_MODE);

	// Display on, cursor and blinking are off
	lcd_on();
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

void lcd_print_P(const char *pstr)
{
	register char c;

	while ( (c = pgm_read_byte(pstr++)) ) {
		lcd_data(c);
	}
}

void lcd_print(const char *str)
{
	register char c;

	while ( (c = *str++) ) {
		lcd_data(c);
	}

}


#endif //LCD_H
