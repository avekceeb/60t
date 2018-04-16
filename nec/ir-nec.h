/************ NEC IR decode lib *****************************/

// this lib from https://github.com/biletnikov/avr-nec-ir-decoder
// adapted to atmega8@12MHz
// Author: Sergei Biletnikov

#ifndef IR_RECEIVER_H_
#define IR_RECEIVER_H_

#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

// port for connecting IR receiver
#define IRR_PORT PORTD
#define IRR_DDR DDRD
#define IRR_PIN PD3
#define IRR_PIN_PORT PIND
#define IRR_INT INT1
// any logical change => interrupt on INT1
#define IRR_INT_MODE ISC10

// init IR receiver pin port
#define init_IRR_PIN() IRR_DDR&=~(1<<IRR_PIN); \
                        IRR_PORT|=(1<<IRR_PIN); \
                        MCUCR|=(1<<IRR_INT_MODE); \
                        GIMSK|=(1<<IRR_INT);

#define NEC_MAX_PACKET_BIT_NUMBER 32

// Packet of data
struct IR_Packet {
    uint8_t addr; // address
    uint8_t addr_inv; // inverted address
    uint8_t command; // command 
    uint8_t command_inv; // inverted command
    uint8_t repeat; // 0 if the packet is not repeat
};

// init receiver
extern void init_receiver();

// check if new data packet received
// 0 - no, otherwise yes
// received_packet is a pointer to the IR_Packet structure to receive the data
// the packet updated only if the function result is not 0
extern uint8_t check_new_packet(struct IR_Packet * received_packet);

#define MAX_DELAY_FOR_REPEAT_COMMAND 100 // in ms
#define MAX_BIT_TRANSMISSION_DELAY 16 // in ms

// packet receiving state
#define PACKET_STATE_NO_PACKET 0
#define PACKET_STATE_READING 1
#define PACKET_STATE_READY 2
#define PACKET_STATE_READ 3

#endif /* IR_RECEIVER_H_ */
