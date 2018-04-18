
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

// ----- sync speed settings ------
//#define speed_diff_threshold 5
//#define speed_sync_step 3
#define speed_default 0xc0
#define speed_min 0xb0
#define speed_max 0xff
#define speed_step 1
#define speed_disbalance 5
//#define speed_upper (speed_max - speed_step)
//#define speed_lower (speed_min + speed_step)
#define speed_upper (0xe0)
#define speed_lower (0xc0)

#define t0_prescale_1 (_BV(CS00))
#define t0_prescale_8 (_BV(CS01))
#define t0_prescale_64 (_BV(CS01)|_BV(CS00))
#define t0_prescale_256 (_BV(CS02))
#define t0_prescale_1024 (_BV(CS02)|_BV(CS00))


// ----- command codes -------------
// IR-remote control for Yamaha CDX4
#include "yamaha-cdx4.h"

#define cmd_invalid 0x00

#define cmd_stop button_play
#define cmd_fwd button_pause
#define cmd_bkw button_stop
#define cmd_left button_begin
#define cmd_right button_end

#define cmd_speed_up button_level_plus
#define cmd_speed_down button_level_minus

#define cmd_speedup_right button_random 
#define cmd_speedup_left  button_repeat

#define cmd_prescale_1  button_1
#define cmd_prescale_8  button_3
#define cmd_prescale_64  button_7
#define cmd_prescale_256  button_9
#define cmd_prescale_1024  button_0

// distance commands:
#define cmd_test_stop button_5
#define cmd_test_fwd button_2
#define cmd_test_bkw button_8
#define cmd_test_left button_4
#define cmd_test_right button_6

#define cmd_trip_plus button_fast_forward
#define cmd_trip_minus button_fast_backward

#define cmd_restart button_open
#define cmd_sync button_sync
#define cmd_program button_prog

#define cmd_display button_index

#define distance_weel_round ((uint16_t)30)
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
    // TODO: bit fields
    uint8_t running;
    uint8_t step_done;
    uint8_t sync;
    uint16_t step_bkw;
    uint16_t step_fwd;
    uint16_t step_turn;
    uint8_t cmds_cnt;
};

#define ReportSize (sizeof(struct Report))
 
    
