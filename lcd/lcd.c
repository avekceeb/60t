#include "lcd.h"

#define F_CPU 12000000UL
#include <util/delay.h>

void sleep20ms() { _delay_ms(20); }
void sleep1ms() { _delay_ms(1); }

void en_strobe() {
    set_bit(en_port, en_bit);
    sleep1ms();
    clear_bit(en_port, en_bit);
    sleep1ms();
}

void lcd_upper_4bit_command(unsigned char command) {
    clear_bit(rs_port, rs_bit);
    data_port = (command & 0xf0) | (data_port & 0x0f);
    en_strobe();
}


void lcd_command(unsigned char command) {
    clear_bit(rs_port, rs_bit);
    data_port = (command & 0xf0) | (data_port & 0x0f);
    en_strobe();
    data_port = ((command<<4) & 0xf0) | (data_port & 0x0f);
    en_strobe();
}


void lcd_data(char byte) {
    set_bit(rs_port, rs_bit);
    data_port = (byte & 0xf0) | (data_port & 0x0f);
    en_strobe();
    data_port = ((byte<<4) & 0xf0) | (data_port & 0x0f);
    en_strobe();
}


void lcd_init() {
    data_dir |= (data_bits) ;
    rs_dir |= _BV(rs_bit);
    en_dir |= _BV(en_bit);

    sleep20ms();
    lcd_upper_4bit_command(lcd_funcset_8bit_2lines_5x8dots);
    sleep20ms();
    lcd_upper_4bit_command(lcd_funcset_8bit_2lines_5x8dots);
    sleep20ms();
    lcd_upper_4bit_command(lcd_funcset_8bit_2lines_5x8dots);
    sleep20ms();
    // switch to 4bit mode
    lcd_upper_4bit_command(lcd_funcset_4bit_2lines_5x8dots);
    sleep20ms();
    // execute commands in 4bit mode
    lcd_command(lcd_funcset_4bit_2lines_5x8dots);
    sleep20ms();
    lcd_command(lcd_display_off);
    sleep20ms();
    lcd_command(lcd_display_clear);
    sleep20ms();
    lcd_command(lcd_entry_mode_cursor_right);
    sleep20ms();
    lcd_command(lcd_display_on);
    sleep20ms();
}
