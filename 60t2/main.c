/*
 * Simple toy car
 * 2 brush motors on H-bridge
 * variant 01.vedi

                               atmega8
                             +---------+
                (RESET) PC6 -| 1     28|- PC5 (ADC5/SCL)
  command ------> (RXD) PD0 -| 2     27|- PC4 (ADC4/SDA)
  [reply] <----x  (TXD) PD1 -| 3     26|- PC3 (ADC3) ----> fwd R
  [obstacle L]x->(INT0) PD2 -| 4     25|- PC2 (ADC2) ----> bkw R
  [obstacle R]x->(INT1) PD3 -| 5     24|- PC1 (ADC1) ----> fwd L
  enc L -----> (XCK/T0) PD4 -| 6     23|- PC0 (ADC0) ----> bkw L
                        VCC -| 7     22|- GND
                        GND -| 8     21|- AREF
          (XTAL1/TOSC1) PB6 -| 9     20|- AVCC
          (XTAL2/TOSC2) PB7 -|10     19|- PB5 (SCK)
  enc R ------->   (T1) PD5 -|11     18|- PB4 (MISO)
  led L <------- (AIN0) PD6 -|12     17|- PB3 (MOSI/OC2) ---> pwm for R & L
  led R <------- (AIN1) PD7 -|13     16|- PB2 (SS/OC1B)
                 (ICP1) PB0 -|14     15|- PB1 (OC1A)
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
#define obsL PD2
#define obsR PD3
#define encL PD4
#define encR PD5
// outputs:
#define txd  PD1
#define fwdR PD3
#define bkwR PD2
#define fwdL PC1
#define bkwL PC0
#define pwm  PB3
#define ledL PD6
#define ledR PD7

#define port_bridge PORTC
#define register_pwm OCR2

// ----- command executions -----
#define bridge_set_direction(__cmd) (port_bridge = __cmd)
#define bridge_set_speed(__speed) (register_pwm = __speed);

#define bridge_stop_left(current)  current &= break_left_mask;\
                            set_direction(current)
#define bridge_stop_right(current) current &= break_right_mask;\
                            set_direction(current)

#define encoder_start_all (TIMSK = (_BV(TOIE0) | _BV(TOIE1)))
// TODO: disable timer, not interrupt
#define encoder_stop_all TIMSK = 0;\
                            TCNT0 = 0; TCNT1L = 0
#define encoder_stop_left TIMSK &= ~(_BV(TOIE0))
#define encoder_stop_right TIMSK &= ~(_BV(TOIE1))

// ---- transmission settings
// rounds of wheel
#define encoder_set_left(x)   TCNT0 = (0xff-(uint8_t)x)
#define encoder_set_right(x)  TCNT1H = 0xff;\
                             TCNT1L = (0xff-(uint8_t)x)

#define led_on_left  (PORTD |= _BV(ledL))
#define led_on_right (PORTD |= _BV(ledR))
#define led_on_all  (PORTD |= (_BV(ledL) | _BV(ledR)))
#define led_off_all  (PORTD &= ~(_BV(ledL) | _BV(ledR)))

// ---- mc state kept in globals ------------------
uint8_t speed;
// direction is actually just a h-bridge combination
uint8_t direction;
uint8_t command;
uint8_t newcommand;
uint8_t distance_bkw, distance_fwd, distance_turn;
// flag for step-by-step execution
uint8_t distance_reached;
//--------------------------------------------------

void sleep1ms() { _delay_ms(1); }
void sleep1s() { _delay_ms(600); }

void bridge_stop() {
    // Note: since there is only one source of EN command
    // to both sides (pwm) all rotations supposed to be
    // symmetrical
    encoder_stop_all;
    direction = break_all_cmd;
    bridge_set_speed(0);
    bridge_set_direction(direction);
}

void bridge_set(uint8_t _dir, uint8_t _speed, uint8_t _dist) {
    direction = _dir;
    speed = _speed;
    if (_dist) {
        encoder_set_left(_dist);
        encoder_set_right(_dist);
        encoder_start_all;
    } else {
        encoder_stop_all;
    }
    bridge_set_speed(speed);
    bridge_set_direction(direction);
}

// shaft encoder L Timer0 - 8bit
ISR (TIMER0_OVF_vect) { // vector 9
    led_on_left;
    bridge_stop();
    distance_reached = 1;
}
// shaft encoder R Timer1 - 16bit
ISR (TIMER1_OVF_vect) { // vector 8
    led_on_right;
    bridge_stop();
    distance_reached = 1;
}

ISR(USART_RXC_vect) { // vector 11
    command = UDR;
    newcommand = 0x01;
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
    for (uint8_t i=0; i<16; i++) {
        distance_reached = 0;
        bridge_set(dance[i].where, speed, dance[i].howmuch);
        while (!distance_reached) {
            if (newcommand) break;
            sleep1ms();
        }
        sleep1s();
    }
}

int main(void) {

startover:
    distance_fwd = distance_fwd_default;
    distance_bkw = distance_bkw_default;
    distance_turn = distance_turn_default;

    // 1) configure IO ports
    // pull-ups for inputs
    PORTD = _BV(rxd) | _BV(obsL) | _BV(obsR) | _BV(encL) | _BV(encR);
    // set outputs:
    DDRD = _BV(ledL) | _BV(ledR);
    DDRB = _BV(pwm);
    DDRC = _BV(fwdR) | _BV(bkwR) | _BV(fwdL) | _BV(bkwL);

    // configure USART
    UBRRH = (uint8_t)(ubrr>>8);
    UBRRL = (uint8_t)ubrr;
    // Enable receiverand Rx Int
    UCSRB = _BV(RXEN) | _BV(RXCIE); // | _BV(TXEN) | TXIE
    // 8 bit ; 1 stop ; no parity ; asynchronous
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);

    // 2) configure timers
    // enable timer interrupt
    //TIMSK = _BV(TOIE0) | _BV(TOIE1);
    // disable int:
    TIMSK = 0;
    // Counter0:
    // External clock source on T0 pin. Clock on rising edge
    TCCR0 = _BV(CS02) | _BV(CS01) | _BV(CS00);
    // Counter1: normal mode
    // External clock source on T1 pin. Clock on rising edge
    TCCR1B = _BV(CS12) | _BV(CS11) | _BV(CS10);
    TCNT1H = 0xff;

    // 4) set phase correct pwm for Counter2 (non-invert)
    // Clear OC2 on Compare Match when up-counting.
    // Set OC2 on Compare Match when downcounting
    // f = Fclk/(N*512) = 23kHz/N N - prescaler
    // lets start with N=1
    TCCR2 = _BV(WGM20)/*pwm*/ | _BV(COM21) | _BV(CS20);
    // TODO: CS22 CS21 CS20 as 000 => stop counter

    // initial state of h-bridge: stop at full speed
    port_bridge = 0;
    distance_reached = 0;
    speed = 0xd0;
    direction = break_all_cmd;
    command = cmd_invalid;
    newcommand = 0x00;

    led_on_all;

    sei();

    while (1) {

        if (newcommand) {

            led_off_all;
            newcommand = 0;
            distance_reached = 0;

            // stop even if it is not movement command
            encoder_stop_all;

            switch(command) {
                case cmd_stop:
                case cmd_test_stop:
                    bridge_stop();
                    break;
                case cmd_fwd:
                    bridge_set(move_fwd_cmd, speed, 0);
                    break;
                case cmd_bkw:
                    bridge_set(move_bkw_cmd, speed, 0);
                    break;
                case cmd_left:
                    bridge_set(turn_left_cmd, speed, 0);
                    break;
                case cmd_right:
                    bridge_set(turn_right_cmd, speed, 0);
                    break;
                case cmd_test_fwd:
                    bridge_set(move_fwd_cmd, speed, distance_fwd);
                    break;
                case cmd_test_bkw:
                    bridge_set(move_bkw_cmd, speed, distance_bkw);
                    break;
                case cmd_test_left:
                    bridge_set(turn_left_cmd, speed, distance_turn);
                    break;
                case cmd_test_right:
                    bridge_set(turn_right_cmd, speed, distance_turn);
                    break;
                case cmd_speedup:
                    if (speed < 0xff) {
                        speed++;
                    }
                    break;
                case cmd_slowdown:
                    if (speed > 0x00) {
                        speed--;
                    }
                    break;
                case cmd_trip_plus:
                    if (distance_fwd<(uint8_t)0xff) distance_fwd++;
                    if (distance_bkw<(uint8_t)0xff) distance_bkw++;
                    if (distance_turn<(uint8_t)0xff)distance_turn++;
                    break;
                case cmd_trip_minus:
                    if (distance_fwd>(uint8_t)0x00) distance_fwd--;
                    if (distance_bkw>(uint8_t)0x00) distance_bkw--;
                    if (distance_turn>(uint8_t)0x00)distance_turn--;
                    break;
                case cmd_restart:
                    goto startover;
                    break;
                case cmd_program:
                    do_dance();
                    break;
            } // switch
            
        } // if new command
        
        sleep1ms();
        
    } // while
    return 0;
}
