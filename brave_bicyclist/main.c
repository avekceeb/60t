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

     Timer 0 - L motor speed control CTC mode
     Timer 1 - 8-bit PWM mode to OC1A,B
     Timer 2 - R motor speed control CTC mode

*/
//------------------------------------------------------------------------------

// TODO: stuck checks
// TODO: wdt reset 

#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "60T.h"
#if use_lcd
    #include "lcd.h"
#endif

//------------------------------------------------------------------------------

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

#define BIT_PWM_L  PB2
#define BIT_PWM_R  PB1

#define PWM_R OCR1B
#define PWM_L OCR1A

#define BRIDGE PORTC

#define MAGIC60T 0x60

//------------------------------------------------------------------------------

#define id_pwm 0
#define id_pwm_max 1
#define id_pwm_min 2
#define id_dticks 3
#define id_divident 4
#define id_divider 5
#define id_step_fwd 6
#define id_step_bkw 7
#define id_step_turn 8
#define parameter_max 8
#define parameter_count (parameter_max+1) 
// TODO: prescale , pause for jitter, pause after command

uint8_t parameters[parameter_count];


void eeprom_save_parameters() {
    uint8_t i;
    for (i=0;i<parameter_count;i++) {
        eeprom_write_byte((uint8_t *)(uint16_t)i, parameters[i]);
    }
    eeprom_write_byte((uint8_t *)(uint16_t)parameter_count, MAGIC60T);
}


uint8_t eeprom_load_parameters() {
    uint8_t i;
    if (MAGIC60T != eeprom_read_byte((const uint8_t *)(uint16_t)parameter_count)) {
        return 0;
    }
    for (i=0;i<parameter_count;i++) {
        parameters[i] = eeprom_read_byte((const uint8_t *)(uint16_t)i);
    }
    return 1;
}


#if use_service_mode
    uint8_t parameter_current;
    uint8_t parameter_value;
    char * parameter_names[parameter_count] = {
        "1: Target PWM   ",
        "2: PWM Maximum  ",
        "3: PWM Minimum  ",
        "4: Ticks Per Ovf",
        "5: Coef Divident",
        "6: Coef Divider ",
        "7: Step Forward ",
        "8: Step Backward",
        "9: Step Turn    ",
    };
#endif

//------------------------------------------------------------------------------
struct Report state;
//------------------------------------------------------------------------------
#if use_lcd
char lcd_up_buffer[18];
char lcd_lo_buffer[18];
char greet[6] = {cyr_p, cyr_r, cyr_i, cyr_v, cyr_e, cyr_t};
char * loaded = "eeprom loaded   ";
char * not_loaded = "eeprom NOT found";
char * saved = "eeprom saved    ";
#endif

struct Step dance[16] = {
    // 1
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_LEFT_CMD, DISTANCE_TURN_DEFAULT*2 },
    // 2
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_LEFT_CMD, DISTANCE_TURN_DEFAULT*2 },
    // 3
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_RIGHT_CMD, DISTANCE_TURN_DEFAULT*2},
    // 4
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_RIGHT_CMD, DISTANCE_TURN_DEFAULT*2 },
    // 5
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_RIGHT_CMD, DISTANCE_TURN_DEFAULT*2 },
    // 6
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_RIGHT_CMD, DISTANCE_TURN_DEFAULT*2 },
    // 7
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_LEFT_CMD, DISTANCE_TURN_DEFAULT*2 },
    // 8
    { MOVE_FWD_CMD, DISTANCE_FWD_DEFAULT },
    { TURN_LEFT_CMD, DISTANCE_TURN_DEFAULT*2 }
};


//------------------------------------------------------------------------------
void pause_for_jitter() {
    _delay_us(20);
}

void pause_after_command() {
    _delay_ms(1);
}

void pause_after_step() {
    _delay_ms(1000);
}

//------------------------------------------------------------------------------
#if use_lcd 
    void _display_state();
    #define display_state _display_state()
#else
    #define display_state
#endif

//------- SPEED CONTROL --------------------------------------------------------

#define enable_ticks  GICR = (_BV(INT0) | _BV(INT1))
#define disable_ticks GICR = 0

void disable_speed_control() {
    TIMSK &= ~(_BV(TOIE0)|_BV(TOIE2));
    TCCR0 = 0;
    TCCR2 = 0;
    TCNT0 = 0;
    TCNT2 = 0;
}

void enable_speed_control() {
    TIMSK |= (_BV(TOIE0)|_BV(TOIE2));
    TCCR0 = T0_PRESCALE_1024;
    TCCR2 = T2_PRESCALE_1024;
    TCNT0 = 0;
    TCNT2 = 0;
}

//------- BRIDGE CONTROL -------------------------------------------------------


void stop() {
    state.running = 0;
    state.step_done = 1;
    disable_ticks;
    disable_speed_control();
    BRIDGE = BREAK_ALL_CMD;
    // TODO: now for debug don't clear last speed
    //PWM_R = 0;
    //PWM_L = 0;
}


void move(uint8_t _direction, uint16_t _distance) {
    state.ticks_l = 0;
    state.ticks_r = 0;
    // TODO:  these 2 are debug:
    state.timer_called_l = 0;
    state.timer_called_r = 0;
    state.distance = _distance;
    state.running = 1;
    state.step_done = 0;
    enable_ticks;
    PWM_R = parameters[id_pwm];
    PWM_L = parameters[id_pwm];
    BRIDGE = _direction;
    enable_speed_control();
}

//------------------------------------------------------------------------------


void do_dance() {
    uint8_t i;
    for (i=0; i<16; i++) {
        // ???
        state.step_done = 0;
        move(dance[i].where, dance[i].howmuch);
        while (!state.step_done) {
            if (state.command) break;
            pause_after_command();
        }
        state.cmds_cnt++;
        pause_after_step();
    }
    stop();
}


//------------------------------------------------------------------------------

#if use_usb
void usart_report_state() {
    uint8_t i = 0;
    uint8_t *p = (uint8_t*)(&state);
    for (; i<ReportSize; i++) {
        while (!(UCSRA & _BV(UDRE)));
        UDR = p[i];
    }
}
#endif

//------------------------------------------------------------------------------

#if use_lcd
void _display_state() {
    cli();
    // TODO: supposing all values are 8bit long
    sprintf (lcd_up_buffer, "cmd=%02x ovf=%02x:%02x",
                            (uint8_t)state.cmds_cnt,
                            (uint8_t)state.timer_called_l,
                            (uint8_t)state.timer_called_r);
    lcd_command(lcd_goto_upper_line);
    for (unsigned char i=0;i<16;i++) {
        lcd_data(lcd_up_buffer[i]);
    }
    sprintf (lcd_lo_buffer, "s=%02x:%02x  t=%02x:%02x",
                            (uint8_t)PWM_L,
                            (uint8_t)PWM_R,
                            (uint8_t)state.ticks_l,
                            (uint8_t)state.ticks_r);
    lcd_command(lcd_goto_lower_line);
    for (unsigned char i=0;i<16;i++) {
        lcd_data(lcd_lo_buffer[i]);
    }
    sei();
}
#endif


//--------- SERVICE MODE -------------------------------------------------------

#if use_service_mode

void display_parameter() {
    cli();
    lcd_command(lcd_goto_upper_line);
    for (unsigned char i=0;i<16;i++) {
        lcd_data(parameter_names[parameter_current][i]);
    }
    sprintf (lcd_lo_buffer, " = 0x%02x         ", parameter_value);
    lcd_command(lcd_goto_lower_line);
    for (unsigned char i=0;i<16;i++) {
        lcd_data(lcd_lo_buffer[i]);
    }
    sei();
}

void load_parameter() {
    parameter_value = parameters[parameter_current];
}

void parameter_list_up() {
    if (parameter_current < parameter_max) {
        parameter_current++;
    } else {
        parameter_current = 0;
    }
}

void parameter_plus() {
    if (parameter_value < 0xff) {
        parameter_value++;
    }
}

void parameter_minus() {
    if (parameter_value > 0) {
        parameter_value--;
    }
}


uint8_t process_service_command(uint8_t cmd) {
    switch (cmd) {
        case CMD_SERVICE_LEAVE:
            state.service_mode = 0;
            lcd_command(lcd_display_clear);
            return 0;
            break;
        case CMD_SERVICE_LIST_UP:
        case CMD_SERVICE_LIST_DOWN:
            parameter_list_up();
            load_parameter();
            break;
        case CMD_SERVICE_VALUE_UP:
            parameter_plus();
            parameters[parameter_current] = parameter_value;
            break;
        case CMD_SERVICE_VALUE_DOWN:
            parameter_minus();
            parameters[parameter_current] = parameter_value;
            break;
        case CMD_SERVICE_EEPROM_SAVE:
            eeprom_save_parameters();
#if use_lcd
            lcd_command(lcd_goto_lower_line);
            for (unsigned char i=0; i<16;i++) {
                lcd_data(saved[i]);
            }
#endif
            break;
        case CMD_SERVICE_EEPROM_LOAD:
            if(eeprom_load_parameters() ) {
#if use_lcd
                lcd_command(lcd_goto_lower_line);
                for (unsigned char i=0; i<16;i++) {
                    lcd_data(loaded[i]);
                }
#endif
            } else {
#if use_lcd
                lcd_command(lcd_goto_lower_line);
                for (unsigned char i=0; i<16;i++) {
                    lcd_data(not_loaded[i]);
                }
#endif
            }
            break;
        default:
            load_parameter();
    }
    display_parameter();
    return 1;
}
#endif


//----------- INIT -------------------------------------------------------------

void parameters_init() {
    if (eeprom_load_parameters()) {
#if use_lcd
        lcd_command(lcd_goto_lower_line);
        for (unsigned char i=0; i<16;i++) {
            lcd_data(loaded[i]);
        }
#endif
    } else {
#if use_lcd
        lcd_command(lcd_goto_lower_line);
        for (unsigned char i=0; i<16;i++) {
            lcd_data(not_loaded[i]);
        }
#endif
        parameters[id_pwm] = SPEED_DEFAULT;
        parameters[id_divident] = DIVIDENT_DEFAULT;
        parameters[id_divider] = DIVIDER_DEFAULT;
        parameters[id_pwm_max] = SPEED_UPPER_DEFAULT;
        parameters[id_pwm_min] = SPEED_LOWER_DEFAULT;
        // this is the count of 'ticks' from encoder between two cons. ovf 
        parameters[id_dticks] = TICKS_PER_OVF;
        parameters[id_step_fwd] = DISTANCE_FWD_DEFAULT;
        parameters[id_step_bkw] = DISTANCE_BKW_DEFAULT;
        parameters[id_step_turn] = DISTANCE_TURN_DEFAULT;
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


//----------- INTERRUPT ROUTINES -----------------------------------------------


// shaft encoder L
ISR(INT0_vect) {
    pause_for_jitter();
    if (state.ticks_l++ >= state.distance) {
        stop();
    }
    state.dticks_l++;
}


// shaft encoder R
ISR(INT1_vect) {
    pause_for_jitter();
    if (state.ticks_r++ >= state.distance) {
        stop();
    }
    state.dticks_r++;
}


// speed sync L timer
ISR(TIMER0_OVF_vect) {
    state.timer_called_l++;
    // TODO: stuck check
    if ((!state.running) || (state.step_done)) {
        return;
    }
    int16_t d = parameters[id_dticks] - state.dticks_l;
    int16_t s = PWM_L; 
    // signed difference between desired and actual speed
    // multiplied by coefficient pwm/ticks
    d = d*parameters[id_divident]/parameters[id_divider];
    s = (uint8_t)(s + d);
    // correct speed if it is inside limits
    if ((parameters[id_pwm_max] > (uint8_t)s) && \
            (parameters[id_pwm_min] < (uint8_t)s)) {
        PWM_L = (uint8_t)s;
    }
    state.dticks_l = 0;
}


// speed sync R timer
ISR(TIMER2_OVF_vect) {
    state.timer_called_r++;
    if ((!state.running) || (state.step_done)) {
        return;
    }
    int16_t d = parameters[id_dticks] - state.dticks_r;
    int16_t s = PWM_R; 
    // signed difference between desired and actual speed
    // multiplied by coefficient pwm/ticks
    d = d*parameters[id_divident]/parameters[id_divider];
    s = (uint8_t)(s + d);
    // correct speed if it is inside limits
    if ((parameters[id_pwm_max] > (uint8_t)s) && \
            (parameters[id_pwm_min] < (uint8_t)s)) {
        PWM_R = (uint8_t)s;
    }
    state.dticks_r = 0;
}


// USART receive
ISR(USART_RXC_vect) {
    state.command = UDR;
}


//------------------------------------------------------------------------------
int main(void) {

startover:

    // --- configure IO ports ------------------------
    // pull-ups for inputs
    PORTD = _BV(rxd) | _BV(encL) | _BV(encR);
    // set outputs:
    DDRB = _BV(BIT_PWM_L) | _BV(BIT_PWM_R);
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

    parameters_init();
    
    // initial state of h-bridge: stop at full speed
    BRIDGE = 0;
    
    state.cmds_cnt = 0;
    state.step_done = 0;
    state.command = 0;
    state.running = 0;
    state.distance = 0;
    state.ticks_l = 0;
    state.ticks_r = 0;
#if use_service_mode
    state.service_mode = 0;
#endif
    state.timer_called_l = 0;
    state.timer_called_r = 0;
    state.dticks_l = 0;
    state.dticks_r = 0;

    sei();

    uint8_t command;
    while (1) {

        if (state.command) { // new command
            
            state.cmds_cnt++;
            command = state.command;
            state.command = 0;
            state.running = 0;
            state.step_done = 0;

#if use_service_mode
            if (state.service_mode) {
                if (process_service_command(command))
                    continue;
                else
                    // TODO:
                    continue;
            }
#endif

            switch(command) {
#if use_service_mode
                case CMD_SERVICE_ENTER:
                    state.service_mode = 1;
                    if (process_service_command(command))
                        continue;
                    break;
                case CMD_SERVICE_LEAVE:
                    state.service_mode = 0;
                    break;
#endif
                case CMD_STOP:
                    stop();
                    break;
                case CMD_FWD:
                    move(MOVE_FWD_CMD, parameters[id_step_fwd]);
                    break;
                case CMD_BKW:
                    move(MOVE_BKW_CMD, parameters[id_step_bkw]);
                    break;
                case CMD_LEFT:
                    move(TURN_LEFT_CMD, parameters[id_step_turn]);
                    break;
                case CMD_RIGHT:
                    move(TURN_RIGHT_CMD,parameters[id_step_turn]);
                    break;
                case CMD_RESTART:
                    goto startover;
                    break;
                case CMD_PROGRAM:
                    do_dance();
                    break;
            } // switch

            // TODO: use_usb
            //usart_report_state();
            pause_after_command();
            
        } // if new command

        if (state.step_done) {
            state.step_done = 0;
            display_state;
        }

    } // while
    return 0;
}
