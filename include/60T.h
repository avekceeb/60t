
// ---- usart settings ------
#define baudrate 9600
// asyncronous normal mode:
#define ubrr (F_CPU/16/baudrate-1)
// 8 bit ; 1 stop ; no parity ; asynchronous
#define usart_mode (_BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0))

#define usb_cmd_set ((uint8_t)0xA0)
#define usb_cmd_get ((uint8_t)0xB0)

// ---- helpers ------
#define set_bit(port,bit)   (port |= _BV(bit))
#define clear_bit(port,bit) (port &= ~(_BV(bit)))

// ---- h-bridge port --------
#define move_fwd_cmd   0b00001010
#define move_bkw_cmd   0b00000101
#define turn_left_cmd  0b00000110
#define turn_right_cmd 0b00001001
#define break_all_cmd  0b00000000

#define break_left_mask 0b11111100
#define break_right_mask 0b11110011

// ----- command codes -------------
// IR-remote control for Yamaha CDX4
#include "yamaha-cdx4.h"

#define cmd_invalid 0x00

#define cmd_stop button_play
#define cmd_fwd button_pause
#define cmd_bkw button_stop
#define cmd_left button_begin
#define cmd_right button_end

#define cmd_speedup_right button_level_plus
#define cmd_speedup_left button_level_minus

// distance commands:
#define cmd_test_stop button_5
#define cmd_test_fwd button_2
#define cmd_test_bkw button_8
#define cmd_test_left button_4
#define cmd_test_right button_6

#define cmd_trip_plus button_fast_forward
#define cmd_trip_minus button_fast_backward

#define cmd_restart button_sync
#define cmd_program button_prog

#define cmd_display button_index

#define distance_weel_round ((uint16_t)12)
#define distance_fwd_default (uint16_t)(distance_weel_round*3)
#define distance_bkw_default (uint16_t)(distance_weel_round*2)
#define distance_turn_default (distance_weel_round)

struct Step {
    uint8_t where;
    uint16_t howmuch;
};

struct Report {
    uint16_t ticks_l;
    uint16_t ticks_r;
    uint8_t speed_l;
    uint8_t speed_r;
    uint8_t direction;
    uint8_t command;
    uint8_t running;
    uint8_t step_done;
    uint16_t step_bkw;
    uint16_t step_fwd;
    uint16_t step_turn;
    uint8_t cmds_cnt;
};

#define ReportSize (sizeof(struct Report))
 
    
