

// --- configurables ----
#define skip_repeat 1
//#define use_usb 0
//#define use_lcd 0
#define use_service_mode 1

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
#define MOVE_FWD_CMD     0b00001010
#define MOVE_BKW_CMD     0b00000101
#define TURN_LEFT_CMD    0b00000110
#define TURN_RIGHT_CMD   0b00001001
#define BREAK_ALL_CMD    0b00000000
//#define BREAK_LEFT_MASK  0b11111100
//#define BREAK_RIGHT_MASK 0b11110011


// ----- sync speed settings ------
#define SPEED_DEFAULT            0xc0
#define SPEED_DISBALANCE_DEFAULT 3
#define SPEED_UPPER_DEFAULT      (0xe0)
#define SPEED_LOWER_DEFAULT      (0xc0)

#define T0_PRESCALE_1    (_BV(CS00))
#define T0_PRESCALE_8    (_BV(CS01))
#define T0_PRESCALE_64   (_BV(CS01)|_BV(CS00))
#define T0_PRESCALE_256  (_BV(CS02))
#define T0_PRESCALE_1024 (_BV(CS02)|_BV(CS00))


// ----- command codes -------------
// IR-remote control for Yamaha CDX4
#include "yamaha-cdx4.h"

#define CMD_INVALID        0x00
#define CMD_SPEED_UP       button_level_plus
#define CMD_SPEED_DOWN     button_level_minus
#define CMD_SPEEDUP_RIGHT  button_random 
#define CMD_SPEEDUP_LEFT   button_repeat
#define CMD_PRESCALE_1     button_1
#define CMD_PRESCALE_8     button_3
#define CMD_PRESCALE_64    button_7
#define CMD_PRESCALE_256   button_9
#define CMD_PRESCALE_1024  button_0


// movement commands:
#define CMD_STOP  button_play
#define CMD_FWD   button_pause
#define CMD_BKW   button_stop
#define CMD_LEFT  button_begin
#define CMD_RIGHT button_end

#define CMD_TEST_STOP  button_5
#define CMD_TEST_FWD   button_2
#define CMD_TEST_BKW   button_8
#define CMD_TEST_LEFT  button_4
#define CMD_TEST_RIGHT button_6

#define CMD_RESTART button_space
#define CMD_PROGRAM button_prog


#define DISTANCE_WEEL_ROUND   ((uint8_t)20)
#define DISTANCE_FWD_DEFAULT  (uint8_t)(DISTANCE_WEEL_ROUND*3)
#define DISTANCE_BKW_DEFAULT  (uint8_t)(DISTANCE_WEEL_ROUND*2)
#define DISTANCE_TURN_DEFAULT (DISTANCE_WEEL_ROUND)


// service mode:
#define CMD_SERVICE_ENTER      button_sync
#define CMD_SERVICE_LEAVE      button_open
#define CMD_SERVICE_LIST_UP    button_pause
#define CMD_SERVICE_LIST_DOWN  button_stop
#define CMD_SERVICE_VALUE_UP   button_level_plus
#define CMD_SERVICE_VALUE_DOWN button_level_minus
#define CMD_SERVICE_WRITE      button_prog


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
    //uint16_t step_bkw;
    //uint16_t step_fwd;
    //uint16_t step_turn;
    uint8_t cmds_cnt;
};

#define ReportSize (sizeof(struct Report))
 
    
