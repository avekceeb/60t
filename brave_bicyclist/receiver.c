/*
                               atmega8
                             +---------+
                (RESET) PC6 -| 1     28|- PC5 (ADC5/SCL)
  status   ---->  (RXD) PD0 -| 2     27|- PC4 (ADC4/SDA)
  command  <----  (TXD) PD1 -| 3     26|- PC3 (ADC3)            
 *USB|sensor --> (INT0) PD2 -| 4     25|- PC2 (ADC2)            
  IR tsop  ----> (INT1) PD3 -| 5     24|- PC1 (ADC1)             
               (XCK/T0) PD4 -| 6     23|- PC0 (ADC0)            
                        VCC -| 7     22|- GND
                        GND -| 8     21|- AREF
          (XTAL1/TOSC1) PB6 -| 9     20|- AVCC
          (XTAL2/TOSC2) PB7 -|10     19|- PB5 (SCK)
                   (T1) PD5 -|11     18|- PB4 (MISO)
                 (AIN0) PD6 -|12     17|- PB3 (MOSI/OC2)                   
                 (AIN1) PD7 -|13     16|- PB2 (SS/OC1B)
  USB D-  <----> (ICP1) PB0 -|14     15|- PB1 (OC1A) <-----> USB D+
                             +---------+

  INT0 could be used as USB interrupt (use_usb=1)
                  or as a sensor interrupt

*/

#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

#include "ir-nec.h"
#include "60T.h"

#if use_usb
    #include "usbdrv.h"
    #include "oddebug.h"
#endif


// ---- connections -----
// inputs:
#if use_usb
    #define usbint PD2
#else
    #define sensor PD2
#endif
#define tsop PD3
#define rxd PD0
// outputs:
#define txd PD1

void usart_transmit(uint8_t data) {
    while (!(UCSRA & _BV(UDRE)));
    UDR = data;
}

static uint8_t replyBuf[ReportSize];

/************************************************************/
// vUSB
#if use_usb

// from host ---> to uC
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len) {
    uchar i = 0;
    while (i < len) {
        // forward command to 'main' uC
        usart_transmit(data[i++]);
    }
    return i;
}

// from uC ---> host
USB_PUBLIC uchar usbFunctionSetup(uchar data[ReportSize]) {
    usbRequest_t *rq = (void *)data;
    usbMsgPtr = (usbMsgPtr_t)replyBuf;
    if (usb_cmd_set == rq->bRequest) {
         // call usbFunctionWrite
        return USB_NO_MSG;
    } else if (usb_cmd_get == rq->bRequest) {
        return ReportSize;
    }
    return 0;
}

#endif

/************************************************************/

uint8_t rx_index;

ISR (USART_RXC_vect) {
    if (rx_index >= ReportSize) {
        rx_index = 0;
    }
    replyBuf[rx_index++] = UDR;
}

#if !use_usb
ISR(INT0_vect) {
    usart_transmit(CMD_STOP);
}
#endif

struct IR_Packet received_packet;

int main(void) {

    rx_index = 0;
    
#if use_usb
    PORTD = 0;
    DDRD = ~(_BV(PD2));
    PORTB = 0;
    DDRB = ~(_BV(PB0) | _BV(PB1)) ;
#else
    // pull-ups for inputs
    PORTD |= _BV(sensor);
    // enable interrupt int0
    GIMSK = _BV(INT0);
    // The falling edge of INT0 generates an interrupt request
    MCUCR = _BV(ISC01);
#endif

    // 4) configure USART
    UBRRH = (uint8_t)(ubrr>>8);
    UBRRL = (uint8_t)ubrr;
    UCSRB = _BV(TXEN) | _BV(RXEN) | _BV(RXCIE);
    UCSRC = usart_mode;

#if use_usb
    usbInit();
#endif

    sei();

    init_receiver();

    while (1) {

#if use_usb
        usbPoll();
#endif
        cli();
        uint8_t check_result = check_new_packet(&received_packet);
        sei();
        if (check_result) {
#if skip_repeat
            if (received_packet.repeat) {
                continue;
            }
#endif
            usart_transmit(received_packet.command);
        }
    }
    return 0;
}
