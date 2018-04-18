#ifndef LCD_H
#define LCD_H

#include <avr/io.h>
#include "wh1602l-ygk-ct.h"

#ifndef set_bit
#define set_bit(port,bit)   (port |= _BV(bit))
#define clear_bit(port,bit) (port &= ~(_BV(bit)))
#endif

// configurables:

#define en_bit PB3
#define en_port PORTB
#define en_dir DDRB

#define rs_bit PB4
#define rs_port PORTB
#define rs_dir DDRB

#define data_port PORTD
#define data_dir DDRD

#define data_bits (_BV(PD4)|_BV(PD5)|_BV(PD6)|_BV(PD7))

void lcd_upper_4bit_command(unsigned char command);

void lcd_command(unsigned char command);

void lcd_data(char byte);

void lcd_init();

#endif
