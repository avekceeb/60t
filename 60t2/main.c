/*

                               atmega8
                             +---------+
                (RESET) PC6 -| 1     28|- PC5 (ADC5/SCL)
  command ------> (RXD) PD0 -| 2     27|- PC4 (ADC4/SDA)
  [reply] <------ (TXD) PD1 -| 3     26|- PC3 (ADC3) ----> fwd R
  enc L -------->(INT0) PD2 -| 4     25|- PC2 (ADC2) ----> bkw R
  enc R -------->(INT1) PD3 -| 5     24|- PC1 (ADC1) ----> fwd L
               (XCK/T0) PD4 -| 6     23|- PC0 (ADC0) ----> bkw L
                        VCC -| 7     22|- GND
                        GND -| 8     21|- AREF
          (XTAL1/TOSC1) PB6 -| 9     20|- AVCC
          (XTAL2/TOSC2) PB7 -|10     19|- PB5 (SCK)
                   (T1) PD5 -|11     18|- PB4 (MISO)
  led L <------- (AIN0) PD6 -|12     17|- PB3 (MOSI/OC2)
  led R <------- (AIN1) PD7 -|13     16|- PB2 (SS/OC1B) ----> pwm L
                 (ICP1) PB0 -|14     15|- PB1 (OC1A)    ----> pwm R
                             +---------+

*/

// TODO: stuck checks
// TODO: wdt reset 
// TODO: if command arrives
// in the middle of 'running' previous ???


#define F_CPU 12000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include "60T.h"

// ---- connections -----
// inputs:
#define rxd  PD0
#define encL PD2
#define encR PD3

// outputs:
#define txd  PD1
#define fwdR PD3
#define bkwR PD2
#define ledL PD6
#define ledR PD7

#define fwdL PC1
#define bkwL PC0

#define pwmL  PB2
#define pwmR  PB1

#define reg_pwm_l OCR1B
#define reg_pwm_r OCR1A

#define port_bridge PORTC

#define bridge_set_direction(__cmd) (port_bridge = __cmd)

#define bridge_set_speed(__l, __r)  reg_pwm_l = __l;\
                                    reg_pwm_r = __r

#define led_on_left  (PORTD |= _BV(ledL))
#define led_on_right (PORTD |= _BV(ledR))
#define led_on_all   (PORTD |= (_BV(ledL) | _BV(ledR)))
#define led_off_all  (PORTD &= ~(_BV(ledL) | _BV(ledR)))

//--------------------------------------------------
struct Report state;
//--------------------------------------------------

uint8_t distance;
uint8_t distance_bkw, distance_fwd, distance_turn;

#define encoder_start_all GICR = (_BV(INT0) | _BV(INT1))
// TODO: disable timer, not interrupt
#define encoder_stop_all GICR = 0;\
                         distance = 0
#define encoder_stop_left GICR &= ~(_BV(INT0))
#define encoder_stop_right GICR &= ~(_BV(INT1))

// ---- transmission settings
#define encoder_set(x)   distance = ((uint8_t)x)

void sleep50us() { _delay_us(50); }
void sleep1ms() { _delay_ms(1); }
void sleep1s() { _delay_ms(600); }

void bridge_stop() {
    // Note: since there is only one source of EN command
    // to both sides (pwm) all rotations supposed to be
    // symmetrical
    state.ticks_l = 0;
    state.ticks_r = 0;
    state.direction = break_all_cmd;
    bridge_set_speed(0, 0);
    bridge_set_direction(state.direction);
}

void bridge_set(uint8_t _dir, uint8_t _speed_l, uint8_t _speed_r, uint8_t _dist) {
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
}

// shaft encoder L Timer0 - 8bit
ISR(INT0_vect) {
    sleep50us();
    if (state.ticks_l++ >= distance) {
        led_on_left;
        bridge_stop();
        state.step_done = 1;
    }
}

// shaft encoder R Timer1 - 16bit
ISR(INT1_vect) {
    sleep50us();
    if (state.ticks_r++ >= distance) {
        led_on_right;
        bridge_stop();
        state.step_done = 1;
    }
}

ISR(USART_RXC_vect) { // vector 11
    state.command = UDR;
    state.running = 0x01;
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
            if (state.running) break;
            sleep1ms();
        }
        sleep1s();
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

int main(void) {

startover:

    // --- configure IO ports ------------------------
    // pull-ups for inputs
    PORTD = _BV(rxd) | _BV(encL) | _BV(encR);
    // set outputs:
    DDRB = _BV(pwmL) | _BV(pwmR);
    DDRC = _BV(fwdR) | _BV(bkwR) | _BV(fwdL) | _BV(bkwL);
    DDRD = _BV(ledL) | _BV(ledR);

    // --- configure encoders -------------------------
    // enable interrupts int0 int1
    GIMSK = _BV(INT0) | _BV(INT1);
    // The falling edge of INT1,2 generates an interrupt request
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
    state.step_done = 0;
    state.speed_l = 0xd0;
    state.speed_r = 0xd0;
    state.direction = break_all_cmd;
    state.command = cmd_invalid;
    state.running = 0x00;

    state.step_bkw = distance_bkw_default;
    state.step_fwd = distance_fwd_default;
    state.step_turn = distance_turn_default;

    led_on_all;

    sei();

    while (1) {

        if (state.running) {

            led_off_all;
            state.running = 0;
            state.step_done = 0;

            // stop even if it is not movement command
            encoder_stop_all;

            switch(state.command) {
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
                    if (state.step_fwd<(uint8_t)0xff)
                        state.step_fwd++;
                    if (state.step_bkw<(uint8_t)0xff)
                        state.step_bkw++;
                    if (state.step_turn<(uint8_t)0xff)
                        state.step_turn++;
                    break;
                case cmd_trip_minus:
                    if (state.step_fwd>(uint8_t)0x00)
                        state.step_fwd--;
                    if (state.step_bkw>(uint8_t)0x00)
                        state.step_bkw--;
                    if (state.step_turn>(uint8_t)0x00)
                        state.step_turn--;
                    break;
                case cmd_restart:
                    goto startover;
                    break;
                case cmd_program:
                    do_dance();
                    break;
            } // switch
            
            usart_report_state();
            
        } // if new command
        
        sleep1ms();
        
    } // while
    return 0;
}
