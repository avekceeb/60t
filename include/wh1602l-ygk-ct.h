#ifndef WH1602L_H
#define WH1602L_H

#define lcd_funcset_4bit_2lines_5x8dots 0b00101100
#define lcd_funcset_8bit_2lines_5x8dots 0b00111100
#define lcd_display_off 0b00001000
#define lcd_display_on_cursor_on_blink_on 0b00001111
#define lcd_display_on 0b00001100
#define lcd_display_clear 0b00000001
#define lcd_entry_mode_cursor_right 0b00000110
#define lcd_home 0b00000010
#define lcd_goto_upper_line 0b10000000
#define lcd_goto_lower_line 0b11000000


#define space   0x20

#define cyr_a   0x61
#define cyr_s   0x63
#define cyr_e   0x65
#define cyr_k   0x6b
#define cyr_o   0x6f
#define cyr_r   0x70
#define cyr_u   0x79

#define cyr_B   0xa0
#define cyr_G   0xa1
#define cyr_YO  0xa2
#define cyr_ZH  0xa3
#define cyr_Z   0xa4
#define cyr_I   0xa5
#define cyr_Y   0xa6
#define cyr_L   0xa7
#define cyr_P   0xa8
#define cyr_U   0xa9
#define cyr_F   0xaa
#define cyr_CH  0xab
#define cyr_SH  0xac
#define cyr_HS  0xad
#define cyr_YY  0xae
#define cyr_YE  0xaf

#define cyr_YU  0xb0
#define cyr_YA  0xb1
#define cyr_b   0xb2
#define cyr_v   0xb3
#define cyr_g   0xb4
#define cyr_yo  0xb5
#define cyr_zh  0xb6
#define cyr_z   0xb7
#define cyr_i   0xb8
#define cyr_y   0xb9
#define cyr_k   0xba
#define cyr_l   0xbb
#define cyr_m   0xbc
#define cyr_n   0xbd
#define cyr_p   0xbe
#define cyr_t   0xbf

#define cyr_ch  0xc0
#define cyr_sh  0xc1
#define cyr_hs  0xc2
#define cyr_yy  0xc3
#define cyr_ss  0xc4
#define cyr_ye  0xc5
#define cyr_yu  0xc6
#define cyr_ya  0xc7

#define cyr_D   0xe0
#define cyr_C   0xe1
#define cyr_Ssh 0xe2
#define cyr_d   0xe3
#define cyr_f   0xe4
#define cyr_c   0xe5
#define cyr_ssh 0xe6

#endif
