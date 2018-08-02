
/*

                               atmega8
                             +---------+
                (RESET) PC6 -| 1     28|- PC5 (ADC5/SCL)
  command x-----> (RXD) PD0 -| 2     27|- PC4 (ADC4/SDA)
  [reply] <-----x (TXD) PD1 -| 3     26|- PC3 (ADC3) ----> fwd R
  enc L x------->(INT0) PD2 -| 4     25|- PC2 (ADC2) ----> bkw R
  enc R x------->(INT1) PD3 -| 5     24|- PC1 (ADC1) ----> fwd L
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

     Timer 0 - L motor speed control CTC mode
     Timer 1 - 8-bit PWM mode to OC1A,B
     Timer 2 - R motor speed control CTC mode

*/

//------------------------------------------------------------------------------

#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>

#if use_lcd
    #include "lcd.h"
#endif

#define BAUD 9600
//#define BAUD 2400
// asyncronous normal mode:
#define MYUBRR (F_CPU/16/BAUD-1)

#define BIT_FWD_R PC3
#define BIT_BKW_R PC2
#define BIT_FWD_L PC1
#define BIT_BKW_L PC0


#define BIT_PWM_R  PB2
#define BIT_PWM_L  PB1

#define PWM_R OCR1B
#define PWM_L OCR1A

#define BRIDGE PORTC


#define CMD_MOVE_FWD     0b00001010
#define CMD_MOVE_BKW     0b00000101
#define CMD_TURN_LEFT    0b00000110
#define CMD_TURN_RIGHT   0b00001001
#define CMD_BREAK_ALL    0b00000000


#define TIMER1_FPWM_ENABLE do{TCCR1B |= _BV(WGM13) | _BV(WGM12) | _BV(CS11);}while(0)
#define TIMER1_FPWM_DISABLE do{TCCR1B |= _BV(WGM13) | _BV(WGM12);}while(0)


#define RX_READY 126
#define RX_CODE  0
#define RX_LEFT  (RX_CODE+1)
#define RX_RIGHT (RX_LEFT+1)
#define RX_DONE  (RX_RIGHT+1)


char rx_data = 0;

// rx is an index inside a rx_buffer
uint8_t rx = 0;
#define rx_buff_sz 16
uint8_t rx_buffer[rx_buff_sz];

uint8_t commands = 0;
uint8_t timer0_ovf = 0;
uint8_t err = '.';


//---------- LC display (optional)----------------------------------------------

#if use_lcd

char lcd_buffer[16];
char msg_privet[6]  = {cyr_p, cyr_r, cyr_i, cyr_v, cyr_e, cyr_t};

void _lcd_print_status(void)
{
    sprintf (lcd_buffer,
        "%02x %c L:%02x R:%02x %c",
        commands,
        rx_buffer[RX_CODE],
        rx_buffer[RX_LEFT],
        rx_buffer[RX_RIGHT],
        err);
    lcd_command(lcd_goto_lower_line);
    for (uint8_t i=0;i<16;i++) {
        lcd_data(lcd_buffer[i]);
    }
}

#define lcd_message(m) do{\
    cli();\
    lcd_command(lcd_goto_upper_line);\
    for(uint8_t i=0;i<16;i++){lcd_data(m[i]);}\
    sei();\
    }while(0)

#define lcd_print_status() _lcd_print_status()

#else

#define lcd_message(m)
#define lcd_print_status()

#endif


//------------------------------------------------------------------------------

void transmit(uint8_t data)
{
    while (!(UCSRA & _BV(UDRE)));
    UDR = data;
}


// drive commands

void vehicle_run(uint8_t bridge, uint8_t pwm_l, uint8_t pwm_r)
{
    TIMER1_FPWM_ENABLE;
    PWM_R = pwm_r;
    PWM_L = pwm_l;
    BRIDGE = bridge;
    // reset stop-guard timer
    timer0_ovf = 0;
    TCNT0 = 0;
}


void vehicle_stop(void)
{
    TIMER1_FPWM_DISABLE;
    PORTB &= ~(_BV(BIT_PWM_L) | _BV(BIT_PWM_R));
    BRIDGE = CMD_BREAK_ALL;
    timer0_ovf = 0;
    TCNT0 = 0;
}


void init_timer0(void)
{
    // 5 sec - maximum time of running single command
    // F_CPU (12e6)
    // ovf t = (12e6 * t) / (1024*255)
    // ovf 5 = 230
    // ovf 4 = 184
    TIMSK |= (_BV(TOIE0) | _BV(TOIE2));
    // prescale 1024
    TCCR0 = _BV(CS02) | _BV(CS00);
    TCNT0 = 0;
}


void init_timer1(void)
{
    // WGM 11 12 13 = Fast PWM top=ICR1 Upadate OCR1* at BOTTOM
    // CS10   = no prescaling => 46875.0 Hz
    // CS11   = prescaling 8  => 5860 Hz
    // COM1*1 = Clear OC1A/OC1B on Compare Match when up-counting.
    //          Set OC1A/OC1B on Compare Match when downcounting.
    TCCR1A |= _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TIMER1_FPWM_ENABLE;
    TCNT1 = 0x00;
    ICR1 =  0xff;
    OCR1A = 0x00;
    OCR1B = 0x00;
}



//----------- Interruptions ----------------------------------------------------

#ifdef __AVR_ATmega8__
    ISR(USART_RXC_vect) {
#else
    ISR(USART_RX_vect) {
#endif
    rx_data = UDR;
    switch (rx) {
        case RX_READY:
            if ('H' == rx_data) {
                rx = RX_CODE;
            } else {
                err = '!';
            }
            break;
        case RX_LEFT:
        case RX_RIGHT:
        case RX_CODE:
            rx_buffer[rx++] = rx_data;
            break;
        case RX_DONE:
            break;
        default:
            err = '*';
    }
}



ISR(TIMER0_OVF_vect)
{
    if (++timer0_ovf >= 220 /* ~4.5 sec */ ) {
        //timer0_ovf = 0;
        vehicle_stop();
    }
}


int main(void)
{

    // pull-ups:
    PORTD |= _BV(PD0);
    // set outputs:
    DDRB = _BV(BIT_PWM_L) | _BV(BIT_PWM_R) | _BV(PB0) /*LED*/;
    DDRC = _BV(BIT_FWD_R) | _BV(BIT_BKW_R) | _BV(BIT_FWD_L) | _BV(BIT_BKW_L);

    // USART TX
    DDRD = _BV(PD1);

    init_timer1();

#if use_lcd
    lcd_init();
    lcd_command(lcd_goto_upper_line);
    for (unsigned char i=0; i<6;i++) {
        lcd_data(msg_privet[i]);
    }
#endif

    // Set baud rate
    UBRRH = (unsigned char)(MYUBRR>>8);
    UBRRL = (unsigned char)MYUBRR;
    // Enable receiver and transmitter and Rx Int
    UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE); // | TXIE
#ifdef __AVR_ATmega8__
    // 8 bit ; 1 stop ; no parity ; asynchronous
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
#else
    UCSRC = (1 << USBS) | (3 << UCSZ0);	// asynchron 8n1
#endif
    // Set frame format: 8data, 2stop bit
    //UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);

    rx = RX_READY;
    
    init_timer0();

    sei();

    while (1) {
        if (RX_DONE == rx) {
            cli();
            if (RX_DONE != rx) {
                goto skip;
            }
            uint8_t code = rx_buffer[RX_CODE];
            uint8_t l = rx_buffer[RX_LEFT];
            uint8_t r = rx_buffer[RX_RIGHT];
            switch (code) {
                case 's': // stop
                    vehicle_stop();
                    break;
                case 'f': // fwd
                    vehicle_run(CMD_MOVE_FWD, l, r);
                    break;
                case 'b': // bkw
                    vehicle_run(CMD_MOVE_BKW, l, r);
                    break;
                case 'l': // left
                    vehicle_run(CMD_TURN_LEFT, l, r);
                    break;
                case 'r': // right
                    vehicle_run(CMD_TURN_RIGHT, l, r);
                    break;
                default:
                    err = '?';
            }
            commands++;
            lcd_print_status();
            err = '.';
            rx = RX_READY;
skip:
            sei();
        }
        // ???
        _delay_ms(200);
    }
}
