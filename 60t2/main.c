/*

                               atmega8
                             +---------+
                (RESET) PC6 -| 1     28|- PC5 (ADC5/SCL)
  command ------> (RXD) PD0 -| 2     27|- PC4 (ADC4/SDA)
  [reply] <------ (TXD) PD1 -| 3     26|- PC3 (ADC3) ----> fwd R (dirs wrong!)
  enc L -------->(INT0) PD2 -| 4     25|- PC2 (ADC2) ----> bkw R
  enc R -------->(INT1) PD3 -| 5     24|- PC1 (ADC1) ----> fwd L
  lcd:d4 <---- (XCK/T0) PD4 -| 6     23|- PC0 (ADC0) ----> bkw L
                        VCC -| 7     22|- GND
                        GND -| 8     21|- AREF
          (XTAL1/TOSC1) PB6 -| 9     20|- AVCC
          (XTAL2/TOSC2) PB7 -|10     19|- PB5 (SCK)
  lcd:d5 <------   (T1) PD5 -|11     18|- PB4 (MISO)     ----> lcd:rs
  lcd:d6 <------ (AIN0) PD6 -|12     17|- PB3 (MOSI/OC2) ----> lcd:en
  lcd:d7 <------ (AIN1) PD7 -|13     16|- PB2 (SS/OC1B) ----> pwm L
                 (ICP1) PB0 -|14     15|- PB1 (OC1A)    ----> pwm R
                             +---------+

*/

// TODO: stuck checks
// TODO: wdt reset 
// TODO: if command arrives
// in the middle of 'running' previous ???

//#ifdef use_lcd
    //#define use_lcd 1
//#endif
#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include "60T.h"
#ifdef use_lcd
    #include "lcd.h"
#endif

// ---- connections -----
// inputs:
#define rxd  PD0
#define encL PD2
#define encR PD3

// outputs:
#define txd  PD1

#define fwdR PC3
#define bkwR PC2

#define fwdL PC1
#define bkwL PC0

#define pwmL  PB2
#define pwmR  PB1

#define reg_pwm_r OCR1B
#define reg_pwm_l OCR1A

#define port_bridge PORTC

#define bridge_set_direction(__cmd) (port_bridge = __cmd)

#define bridge_set_speed(__l, __r)  reg_pwm_l = __l;\
                                    reg_pwm_r = __r

//--------------------------------------------------
struct Report state;
//--------------------------------------------------
uint8_t prescale_for_t0;
uint16_t timer0_ovf;

uint16_t distance;
uint16_t distance_bkw, distance_fwd, distance_turn;

#define encoder_start_all GICR = (_BV(INT0) | _BV(INT1))
#define encoder_stop_all GICR = 0;\
                         distance = 0
#define encoder_stop_left GICR &= ~(_BV(INT0))
#define encoder_stop_right GICR &= ~(_BV(INT1))

#define encoder_set(x)   distance = ((uint16_t)x)

void pause_for_jitter() {
    _delay_us(20);
}

void pause_after_command() {
    _delay_ms(1);
}

void pause_after_step() {
    _delay_ms(1000);
}

#ifdef use_lcd 
    void _display_state();
    #define display_state _display_state()
#else
    #define display_state
#endif

void disable_sync() {
    TIMSK &= ~(_BV(TOIE0));
    TCCR0 = 0;
    TCNT0 = 0;
}

void enable_sync() {
    TIMSK |= _BV(TOIE0);
    TCCR0 = prescale_for_t0;
    TCNT0 = 0;
}

// timer 0
void switch_speed_sync() {
    if (state.sync) {
        disable_sync();
        state.sync = 0;
    } else {
        enable_sync();
        state.sync = 1;
    }
}

void bridge_stop() {
    state.running = 0;
    state.ticks_l = 0;
    state.ticks_r = 0;
    state.direction = break_all_cmd;
    bridge_set_speed(0, 0);
    bridge_set_direction(state.direction);
    disable_sync();

}

void bridge_set(uint8_t _dir, uint8_t _speed_l, uint8_t _speed_r, uint16_t _dist) {
    state.direction = _dir;
    state.speed_l = _speed_l;
    state.speed_r = _speed_r;
    if (_dist) {
        encoder_set(_dist);
        encoder_start_all;
    } else {
        encoder_stop_all;
    }
    bridge_set_speed(state.speed_l, state.speed_r);
    bridge_set_direction(state.direction);
    state.running = 1;
    if (state.sync) {
        enable_sync();
    }
}

// shaft encoder L
ISR(INT0_vect) {
    pause_for_jitter();
    if (state.ticks_l++ >= distance) {
        // here we disable it quickly
        // this also be repeated in bridge_stop
        disable_sync();
        display_state;
        bridge_stop();
        state.step_done = 1;
    }
}

// shaft encoder R
ISR(INT1_vect) {
    pause_for_jitter();
    if (state.ticks_r++ >= distance) {
        // here we disable it quickly
        // this also be repeated in bridge_stop
        disable_sync();
        display_state;
        bridge_stop();
        state.step_done = 1;
    }
}

ISR(USART_RXC_vect) {
    // TODO: try blocking mode - hold before current command finishes
    state.command = UDR;
}

struct Step dance[16] = {
    // 1
    { move_fwd_cmd, distance_fwd_default },
    { turn_left_cmd, distance_turn_default*2 },
    // 2
    { move_fwd_cmd, distance_fwd_default },
    { turn_left_cmd, distance_turn_default*2 },
    // 3
    { move_fwd_cmd, distance_fwd_default },
    { turn_right_cmd, distance_turn_default*2},
    // 4
    { move_fwd_cmd, distance_fwd_default },
    { turn_right_cmd, distance_turn_default*2 },
    // 5
    { move_fwd_cmd, distance_fwd_default },
    { turn_right_cmd, distance_turn_default*2 },
    // 6
    { move_fwd_cmd, distance_fwd_default },
    { turn_right_cmd, distance_turn_default*2 },
    // 7
    { move_fwd_cmd, distance_fwd_default },
    { turn_left_cmd, distance_turn_default*2 },
    // 8
    { move_fwd_cmd, distance_fwd_default },
    { turn_left_cmd, distance_turn_default*2 }
};

void do_dance() {
    uint8_t i;
    for (i=0; i<16; i++) {
        state.step_done = 0;
        bridge_set(dance[i].where, state.speed_l, state.speed_r, dance[i].howmuch);
        while (!state.step_done) {
            if (state.command) break;
            pause_after_command();
        }
        state.cmds_cnt++;
        pause_after_step();
    }
}

void usart_report_state() {
    uint8_t i = 0;
    uint8_t *p = (uint8_t*)(&state);
    for (; i<ReportSize; i++) {
        while (!(UCSRA & _BV(UDRE)));
        UDR = p[i];
    }
}

void timer1_init(void) {
    // WGM 11 12 13 = Fast PWM top=ICR1 Upadate OCR1* at BOTTOM
    // CS10 = no prescaling
    // COM1*1 = Clear OC1A/OC1B on Compare Match when up-counting.
    //          Set OC1A/OC1B on Compare Match when downcounting.
    TCCR1A |= _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B |= _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    TCNT1 = 0x00;
    ICR1 =  0xff;
    OCR1A = 0x00;
    OCR1B = 0x00;
}

// ---------------------------------------------------------
#if use_lcd

char lcd_up_buffer[18];
char lcd_lo_buffer[18];
char greet[6] = {cyr_p, cyr_r, cyr_i, cyr_v, cyr_e, cyr_t};

void _display_state() {
    sprintf (lcd_up_buffer, "c:%02x o:%04x %c%c%c ",
                            state.cmds_cnt,
                            timer0_ovf,
                            state.running ? 'R' : ' ',
                            state.step_done ? 'D' : ' ',
                            state.sync ? 'S' : ' '
                            );
    lcd_command(lcd_goto_upper_line);
    for (unsigned char i=0;i<16;i++) {
        lcd_data(lcd_up_buffer[i]);
    }
    sprintf (lcd_lo_buffer, "%02x:%02x  %04x:%04x",
                            state.speed_l,
                            state.speed_r,
                            state.ticks_l,
                            state.ticks_r);
    lcd_command(lcd_goto_lower_line);
    for (unsigned char i=0;i<16;i++) {
        lcd_data(lcd_lo_buffer[i]);
    }
}

#endif

// speed sync timer
ISR(TIMER0_OVF_vect) {
    timer0_ovf++;
    if ((!state.running) || (!state.sync)) {
        return;
    }
    int16_t dif = state.ticks_r - state.ticks_l;
    if (dif > speed_disbalance) {
        if (state.speed_l < speed_upper) {
            state.speed_l += speed_step;
        }
        if (state.speed_r > speed_lower) {
            state.speed_r -= speed_step;
        }
    } else if (dif < -speed_disbalance) {
        if (state.speed_l > speed_lower) {
            state.speed_l -= speed_step;
        } 
        if (state.speed_r < speed_upper) {
            state.speed_r += speed_step;
        }
    } else {
        return;
    }
    bridge_set_speed(state.speed_l, state.speed_r);
}

int main(void) {

startover:

    // --- configure IO ports ------------------------
    // pull-ups for inputs
    PORTD = _BV(rxd) | _BV(encL) | _BV(encR);
    // set outputs:
    DDRB = _BV(pwmL) | _BV(pwmR);
    DDRC = _BV(fwdR) | _BV(bkwR) | _BV(fwdL) | _BV(bkwL);

#if use_lcd
    lcd_init();
    lcd_command(lcd_goto_upper_line);
    for (unsigned char i=0; i<6;i++) {
        lcd_data(greet[i]);
    }
#endif

    // --- configure encoders -------------------------
    // enable interrupts int0 int1
    GIMSK = _BV(INT0) | _BV(INT1);
    // The falling edge of INT0 and INT1 generates an interrupt request
    MCUCR = _BV(ISC11) | _BV(ISC01);

    // ---- configure USART ---------------------------
    UBRRH = (uint8_t)(ubrr>>8);
    UBRRL = (uint8_t)ubrr;
    // Enable Rx, Tx, RxInt
    UCSRB = _BV(RXEN) | _BV(RXCIE) | _BV(TXEN); // | TXIE
    // 8 bit ; 1 stop ; no parity ; asynchronous
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);

    // ---- configure timers for PWM ---------------------
    timer1_init();
    
    // initial state of h-bridge: stop at full speed
    port_bridge = 0;
    
    state.cmds_cnt = 0;
    state.step_done = 0;
    state.speed_l = speed_default;
    state.speed_r = speed_default;
    state.direction = break_all_cmd;
    state.command = 0;
    state.running = 0;
    state.sync = 0;
    state.step_bkw = distance_bkw_default;
    state.step_fwd = distance_fwd_default;
    state.step_turn = distance_turn_default;

    prescale_for_t0 = t0_prescale_1024;
    timer0_ovf = 0;

    sei();

    uint8_t command;
    while (1) {

        if (state.command) { // new command
            
            state.cmds_cnt++;
            command = state.command;
            state.command = 0;
            state.running = 0;
            state.step_done = 0;

            // stop even if it is not movement command
            encoder_stop_all;

            switch(command) {
                case cmd_display:
                    break;
                case cmd_stop:
                case cmd_test_stop:
                    bridge_stop();
                    break;
                case cmd_fwd:
                    bridge_set(move_fwd_cmd, state.speed_l, state.speed_r, 0);
                    break;
                case cmd_bkw:
                    bridge_set(move_bkw_cmd, state.speed_l, state.speed_r, 0);
                    break;
                case cmd_left:
                    bridge_set(turn_left_cmd, state.speed_l, state.speed_r, 0);
                    break;
                case cmd_right:
                    bridge_set(turn_right_cmd, state.speed_l, state.speed_r, 0);
                    break;
                case cmd_test_fwd:
                    bridge_set(move_fwd_cmd, state.speed_l, state.speed_r, state.step_fwd);
                    break;
                case cmd_test_bkw:
                    bridge_set(move_bkw_cmd, state.speed_l, state.speed_r, state.step_bkw);
                    break;
                case cmd_test_left:
                    bridge_set(turn_left_cmd, state.speed_l, state.speed_r, state.step_turn);
                    break;
                case cmd_test_right:
                    bridge_set(turn_right_cmd, state.speed_l, state.speed_r, state.step_turn);
                    break;
                case cmd_speed_up:
                    if (state.speed_l < 0xff) {
                        state.speed_l++;
                    }
                    if (state.speed_r < 0xff) {
                        state.speed_r++;
                    }
                    break;
                case cmd_speed_down:
                    if (state.speed_l > 0) {
                        state.speed_l--;
                    }
                    if (state.speed_r > 0) {
                        state.speed_r--;
                    }
                    break;
                case cmd_speedup_left:
                    if (state.speed_l < 0xff) {
                        state.speed_l++;
                    }
                    if (state.speed_r > 0x00) {
                        state.speed_r--;
                    }
                    break;
                case cmd_speedup_right:
                    if (state.speed_r < 0xff) {
                        state.speed_r++;
                    }
                    if (state.speed_l > 0x00) {
                        state.speed_l--;
                    }
                    break;
                case cmd_trip_plus:
                    if (state.step_fwd<(uint16_t)0xffff)
                        state.step_fwd++;
                    if (state.step_bkw<(uint16_t)0xffff)
                        state.step_bkw++;
                    if (state.step_turn<(uint16_t)0xffff)
                        state.step_turn++;
                    break;
                case cmd_trip_minus:
                    if (state.step_fwd>0)
                        state.step_fwd--;
                    if (state.step_bkw>0)
                        state.step_bkw--;
                    if (state.step_turn>0)
                        state.step_turn--;
                    break;
                case cmd_sync:
                    switch_speed_sync();
                    break;
                case cmd_prescale_1:
                    prescale_for_t0 = t0_prescale_1;
                    break;
                case cmd_prescale_8:
                    prescale_for_t0 = t0_prescale_8;
                    break;
                case cmd_prescale_64:
                    prescale_for_t0 = t0_prescale_64;
                    break;
                case cmd_prescale_256:
                    prescale_for_t0 = t0_prescale_256;
                    break;
                case cmd_prescale_1024:
                    prescale_for_t0 = t0_prescale_1024;
                    break;
                case cmd_restart:
                    goto startover;
                    break;
                case cmd_program:
                    do_dance();
                    break;
            } // switch

            TCCR0 = prescale_for_t0;
            // TODO: use_usb
            //usart_report_state();

            display_state;
            
            } // if new command
        
        pause_after_command();
        
    } // while
    return 0;
}
