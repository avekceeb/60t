
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


// settings:

#define PWM_MAX 80
#define PWM_MIN 32


uint8_t rx_data = 0;
uint8_t command = 0;

void transmit(uint8_t data) {
    while (!(UCSRA & _BV(UDRE)));
    UDR = data;
}


#if use_lcd
char lcd_up_buffer[16];
char lcd_lo_buffer[16];
char msg_privet[6]  = {cyr_p, cyr_r, cyr_i, cyr_v, cyr_e, cyr_t};
char * msg_forward  = "^^^^ forward ^^^";
char * msg_backward = "vvv backward vvv";
char * msg_left     = "<<<<< left <<<<<";
char * msg_right    = ">>>> right >>>>>";
void _lcd_print_code(uint8_t c) {
    cli();
    sprintf (lcd_lo_buffer, "0x%02x           ", c);
    lcd_command(lcd_goto_lower_line);
    for (unsigned char i=0;i<15;i++) {
        lcd_data(lcd_lo_buffer[i]);
    }
    sei();
}
#define lcd_message(m) do{\
    cli();\
    lcd_command(lcd_goto_upper_line);\
    for (unsigned char i=0; i<16;i++){lcd_data(m[i]);}\
    sei();\
    }while(0)
#define lcd_print_code(c) _lcd_print_code(c)
#else
#define lcd_message(m)
#define lcd_print_code(c)
#endif


void pause_run() {
    _delay_ms(3500);
}


void pause_stop() {
    _delay_ms(1500);
}


void pause_small() {
    _delay_ms(500);
}


void timer1_init(void) {
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


// interruptions

#ifdef __AVR_ATmega8__
    ISR(USART_RXC_vect) {
#else
    ISR(USART_RX_vect) {
#endif
    rx_data = UDR;
    command = rx_data;
    lcd_print_code(command);
    transmit(command);
    transmit('\r');
    transmit('\n');
}


// drive commands

void vehicle_run(uint8_t bridge, uint8_t pwm_l, uint8_t pwm_r) {
    TIMER1_FPWM_ENABLE;
    PWM_R = pwm_r;
    PWM_L = pwm_l;
    BRIDGE = bridge;
}


void vehicle_stop() {
    TIMER1_FPWM_DISABLE;
    PORTB &= ~(_BV(BIT_PWM_L) | _BV(BIT_PWM_R));
    BRIDGE = CMD_BREAK_ALL;
}

#define VEHICLE_FWD vehicle_run(CMD_MOVE_FWD, PWM_MAX, PWM_MAX)
#define VEHICLE_BKW vehicle_run(CMD_MOVE_BKW, PWM_MAX, PWM_MAX)
#define VEHICLE_LFT vehicle_run(CMD_TURN_LEFT, PWM_MAX, PWM_MAX)
#define VEHICLE_RGH vehicle_run(CMD_TURN_RIGHT, PWM_MAX, PWM_MAX)


// main

int main(void) {

    // pull-ups:
    PORTD |= _BV(PD0);
    // set outputs:
    DDRB = _BV(BIT_PWM_L) | _BV(BIT_PWM_R) | _BV(PB0) /*LED*/;
    DDRC = _BV(BIT_FWD_R) | _BV(BIT_BKW_R) | _BV(BIT_FWD_L) | _BV(BIT_BKW_L);

    // USART TX
    DDRD = _BV(PD1);

    timer1_init();

#if use_lcd
    lcd_init();
    lcd_command(lcd_goto_upper_line);
    for (unsigned char i=0; i<6;i++) {
        lcd_data(msg_privet[i]);
    }
#endif

    /* Set baud rate */
    UBRRH = (unsigned char)(MYUBRR>>8);
    UBRRL = (unsigned char)MYUBRR;
    /* Enable receiver and transmitter and Rx Int*/
    UCSRB = _BV(RXEN) | _BV(TXEN) | _BV(RXCIE); // | TXIE
#ifdef __AVR_ATmega8__
    // 8 bit ; 1 stop ; no parity ; asynchronous
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
#else
    UCSRC = (1 << USBS) | (3 << UCSZ0);	// asynchron 8n1
#endif
    /* Set frame format: 8data, 2stop bit */
    //UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
    
    sei();

    while(1) {
        switch (command) {
            case 0x73: // stop
            case 0x20:
                vehicle_stop();
                break;
            case 0x66: // fwd
                VEHICLE_FWD;
                break;
            case 0x62: // bkw
                VEHICLE_BKW;
                break;
            case 0x6c: // left
                VEHICLE_LFT;
                break;
            case 0x72: // right
                VEHICLE_RGH;
                break;
        }
        command = 0;
        pause_small();
    }
}
