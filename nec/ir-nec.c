#include "ir-nec.h"

uint8_t packet_reading_state = PACKET_STATE_NO_PACKET;

// read bits counter
// 0 - the packet receiving was not started
// after the packet received, this counter is set to 0
uint8_t read_bit_counter = 0;

// receiving packet of data
struct IR_Packet packet;
// pulse and delay length
uint16_t pulse_time = 0;
uint16_t pause_time = 0;

// reset packet data
void reset_packet() {
    packet.addr = 0;
    packet.addr_inv = 0;
    packet.command = 0;
    packet.command_inv = 0;
    packet.repeat = 0;
    read_bit_counter = 0;
    packet_reading_state = PACKET_STATE_NO_PACKET;
}

// Start timer in ms
void start_IR_timer(uint8_t time_ms) {
    TCCR1A=0x0;
    TCNT1=0x0;
    // max resolution is 4 microseconds
    // it makes delay = 16 ms when Fcpu = 16 Mhz,
    // it makes the delay enough for reading each bit according to NEC
    //OCR1A = ((uint32_t) time_ms * 1000)/4;
    // for 12MHz:
    OCR1A = ((uint16_t)(time_ms * 187.5));
    TCCR1B |= (1<<WGM12); // CTC mode -> TCNT1 = OCR1A
    TCCR1B |= (1<<CS10)|(1<<CS11); // prescaler 64
    TIMSK |= (1<<OCIE1A); // allow interrupts
}

void stop_IR_timer() {
    TCCR1B &= ~(1<<CS10);
    TCCR1B &= ~(1<<CS11);
    TCCR1B &= ~(1<<CS12);
}

void reset_IR_receiver() {
    stop_IR_timer();
    reset_packet();
    pulse_time = 0;
    pause_time = 0;
}

void on_start_bit() {
    // new packet will be received
    reset_packet();
    packet_reading_state = PACKET_STATE_READING;
}

// the packet is received successfully and ready for further processing
void on_new_packet_received() {
    packet_reading_state = PACKET_STATE_READY;
    start_IR_timer(MAX_DELAY_FOR_REPEAT_COMMAND); // start timer and wait repeat command
}

void on_data_bit(uint8_t bit) {
    if (read_bit_counter < NEC_MAX_PACKET_BIT_NUMBER && packet_reading_state == PACKET_STATE_READING)
    {
        if (bit) {
            if (read_bit_counter < 8) {
                // address reading
                packet.addr |= (1<<read_bit_counter);
            } else if (read_bit_counter >=8 && read_bit_counter < 16) {
                // inverting address reading
                packet.addr_inv |= (1<<(read_bit_counter - 8));
            } else if (read_bit_counter >= 16 && read_bit_counter < 24) {
                // command reading
                packet.command |= (1<<(read_bit_counter - 16));
            } else if (read_bit_counter >= 24) {
                // inverting command reading
                packet.command_inv |= (1<<(read_bit_counter - 24));
            }
        }
        
        read_bit_counter++;
        
        if (read_bit_counter == NEC_MAX_PACKET_BIT_NUMBER) {
            // the packet is read, validate
            if (((packet.addr + packet.addr_inv) == 0xFF) &&
                ((packet.command + packet.command_inv) == 0xFF))
            {
                // new valid packet is received
                on_new_packet_received();
            }
        }
    }
}

void on_repeat_command() {
    if (packet_reading_state == PACKET_STATE_READY || packet_reading_state == PACKET_STATE_READ)
    {
        if (packet.repeat < 255) {
            packet.repeat++; // repeat counter up
        }
        packet_reading_state = PACKET_STATE_READY;
        
        start_IR_timer(MAX_DELAY_FOR_REPEAT_COMMAND); // wait next repeat command
    } else {
        // problem, invalid protocol, reset receiver
        reset_IR_receiver();
        reset_packet();
    }
}

void read_chunk() {
    if (pulse_time > 0 && pause_time > 0) {
        // pulse 7 ms (1750) - 11 ms (2750)
        if (pulse_time > 1313 && pulse_time < 2063) {
            // pause 3.2 ms (800) - 6 ms (1500) 
            if (pause_time > 600 && pause_time < 1125) {
                // start bit
                on_start_bit();
            } 
            // pause 1.6 ms (400) - 3.2 ms (800)
            else if (pause_time > 300 && pause_time <= 600) {
                // command repeat
                on_repeat_command();
            }
        }
        // pulse 360 microseconds (90) - 760 microseconds (190)
        else if (pulse_time > 68 && pulse_time < 143) {
            // pause 1.5 ms (375) - 1.9 ms (475)
            if (pause_time > 281 && pause_time < 356) {
                // data bit = 1
                on_data_bit(1);
            } // pause 360 microseconds (90) - 760 microseconds (190)
            else if (pause_time > 68 && pause_time < 143) {
                // data bit = 0
                on_data_bit(0);
            }
        }
    }
}

// check if new data packet received
// 0 - no, otherwise yes
// received_packet is a pointer to the IR_Packet structure to receive the data
// the packet updated only if the function result is not 0
uint8_t check_new_packet(struct IR_Packet * received_packet) {
    if (packet_reading_state == PACKET_STATE_READY) {
        packet_reading_state = PACKET_STATE_READ;
        received_packet->addr = packet.addr;
        received_packet->addr_inv = packet.addr_inv;
        received_packet->command = packet.command;
        received_packet->command_inv = packet.command_inv;
        received_packet->repeat = packet.repeat;
        return 1;
    }
    return 0;
}

// init receiver
void init_receiver() {
    cli();
    // IR port init
    init_IRR_PIN();
    reset_IR_receiver();
    reset_packet();
    sei();
}

ISR(INT1_vect) {
    uint8_t rising_edge = (IRR_PIN_PORT & (1<<IRR_PIN));

    if (rising_edge) {
        // rising edge interrupt, read the counter value -> duration of pulse
        pulse_time = TCNT1;
        // reset counter
        TCNT1 = 0;
    } else {
        // falling edge interrupt
        if (pulse_time == 0) {
            // new data chunk receiving, start timer to handle problem packets
            start_IR_timer(MAX_BIT_TRANSMISSION_DELAY);
        } else {
            // keep pause duration
            pause_time = TCNT1;
            // reset counter
            TCNT1 = 0;
            // read the piece of data received
            read_chunk();
            pulse_time = 0;
            pause_time = 0;
        }
    }
}

ISR(TIMER1_COMPA_vect) {
    // reset IR receiver on the timer interrupt
    reset_IR_receiver(); 
}
