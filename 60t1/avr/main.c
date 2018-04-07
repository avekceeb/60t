#include <avr/io.h>
#include <avr/interrupt.h>

#if 0
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#endif

#include "usbdrv.h"
#include "oddebug.h"
#include <util/delay.h>

#include "60t.h"

/*
 * PB0          <-> USB D-
 * PB1          <-> USB D+
 * PB2 (OCOA)   --> right pwm (enable) channel 1.0
 * PB3          --> aux/check
 * PB4 (PCINT4) <-- front button 
 * PB5 (PCINT5) <-- back button
 * PB6          --> right bridge 1 - channel 1.1 
 * PB7          --> rigth bridge 2 - channel 1.2
 * PD0          --> left bridge 1 - channel 2.1
 * PD1          --> left bridge 2 - channel 2.2
 * PD2 (INT0)   <-- USB
 * PD3
 * PD4
 * PD5 (OC0B)   --> left pwm (enable) channel 2.0
 * PD6
 */

#define sensorinterrupts 0
#define greeting 0
#define timer1interrupt 1

static uchar replyBuf[report_size];

/*
 * psevdo-structura sostojanie mashiny 
 */
static uchar front_tentacle;
static uchar rear_tentacle;
static uchar timer1flag;
static uchar unused2;


static uchar i;

#if sensorinterrupts
ISR(PCINT_vect) {
    OCR0A = 0;
    OCR0B = 0;
    //PORTB |= _BV(PB3);
    _delay_ms(500);
    if (PINB & _BV(PB4)) { // front
        front_tentacle = 1;
    } else if (PINB & _BV(PB5)) { // back
        rear_tentacle = 1;
    }
    //PORTB &= ~_BV(PB3);
}
#endif

#if timer1interrupt
ISR(TIMER1_OVF_vect) {
    if (timer1flag) {
        PORTB &= ~_BV(PB3);
        timer1flag = 0;
    } else {
        PORTB |= _BV(PB3);
        timer1flag = 1;
    }
}
#endif

/*
 * uchar ch #
 * uchar p0 (pwm)
 * uchar p1
 * uchar p2
 */
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len) {

    i = 0;
    
    while (i < len ) {
    
        switch (data[i]) {
        
            // channel 1
            case 1:
                OCR0A = data[i+1];
                if (data[i+2]) {
                    PORTB |= _BV(PB6);
                } else {
                    PORTB &= ~_BV(PB6);
                }
                if (data[i+3]) {
                    PORTB |= _BV(PB7);
                } else {
                    PORTB &= ~_BV(PB7);
                }
                break;
            
            //channel 2
            case 2:
                OCR0B = data[i+1];
                if (data[i+2]) {
                    PORTD |= _BV(PD0);
                } else {
                    PORTD &= ~_BV(PD0);
                }
                if (data[i+3]) {
                    PORTD |= _BV(PD1);
                } else {
                    PORTD &= ~_BV(PD1);
                }
                break;
        }
        i += 4;
	}
    return i;
		
}

/*
 * micro-management variant:
 * each command contains detailed instructions for executor
 */

USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
	
    usbRequest_t *rq = (void *)data;
	
    usbMsgPtr = (usbMsgPtr_t)replyBuf;

    if (cmd_set == rq->bRequest) {

		return USB_NO_MSG; // usbFunctionWrite
    
    } else if (cmd_get == rq->bRequest) {
    
        replyBuf[0] = 'O';
        replyBuf[1] = 'K';
        replyBuf[2] = '!';
        replyBuf[3] = '\0';
        //replyBuf[2] = front_tentacle;
        //replyBuf[3] = rear_tentacle;
        //front_tentacle = 0;
        //rear_tentacle = 0;
        return report_size;
    
    }
	return 0;
}

int main(void) {

    PORTD = 0;
    DDRD = ~(_BV(PD2));

    PORTB = 0;
    DDRB = ~(_BV(PB0) | _BV(PB1) | _BV(PB4) | _BV(PB5)) ;
    
    /*
     * PWM, Phase Correct
     * Clear OC0A on Compare Match when up-counting.
     * Set OC0A on Compare Match when down-counting.
     */
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);

    /*
     * 12000000.0 / (256 * 2 * 8) = 2929.6875
     * Prescaler = 8
     */
    TCCR0B = (1 << CS01);

    /*
     * disable analog comparator
     */
    ACSR = 0x80;

#if sensorinterrupts 
    /*
     * Enable +2 interrupts
     */
    GIMSK |= _BV(PCIE);
    PCMSK = _BV(PCINT4) | _BV(PCINT5); 
#endif

#if timer1interrupt
    // every 5 seconds
    TCCR1B = _BV(CS12) | _BV(CS10);  // prescale /1024
    TIMSK |= _BV(TOIE1); // enable overflow interrupt
#endif

#if greeting
    for (i=0;i<3;i++) {
        PORTB |= _BV(PB3);
        _delay_ms(500);
        PORTB &= ~_BV(PB3);
        _delay_ms(500);
    }
#endif

    usbInit();

    sei();
    
    for (;;) {
        usbPoll();
    }
    
    return 0;
}

