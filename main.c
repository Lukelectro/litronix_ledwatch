/*
 * File:   main.c
 * Author: lucas
 *
 * Created 2024-03-22
 */

// // TODO misschien is PCint gebruiken om wakker te worden ook iets voor neonwatch? En het versimpelde data-ontvangen ook?
// -- display ON 5 mA. Sleep mode 0.1mA 
// Wakes on pin change interrupt on light sensor pin. Less accurate, perhaps a bit difficult to trigger in low light conditions, but very economic/low power

#include <xc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "rtc.h"
#include <util/crc16.h> // using crc8, contained therein.
#include <avr/sleep.h>
#include <avr/power.h>

#define F_CPU 1e6 // aactualy run at 1 MHz, probably fast enough
#define LDTHRESH 150 // 0-255 light/dark threshold for setting time /data reception. Adapt for screen brightness.


FUSES = {
    .low = 0x62, // 0x62=INTRC 8 MHZ/8 =1Mhz) 3V is enough for 8 MHz (2.7V for up to 10, even), that would be E2 = INTRC 8MHZ/1 = 8MHZ.
    .high = 0xD9, // D9=default. to enable debugwire use 0x99
    .extended = 0xFF, // geen BOD
};
LOCKBITS = 0xFF; // {LB=PROG_VER_DISABLED, BLB0=LPM_SPM_DISABLE, BLB1=LPM_SPM_DISABLE}

uint8_t read_time_optical();
void display_time(uint8_t, uint8_t); // hour, minute
void display_date(uint8_t, uint8_t, uint8_t, uint16_t); // day-of-week, day, month, year
void waitabit(); // waits 1 tick of TCNT2, the RTC timer ticking at 256 Hz. So waits about 4ms.
void waitalongbit(); // waits multiple ticks, about 1 s;
void displayRaw(uint16_t, uint8_t);
uint16_t NumTo7Seg(uint8_t num);
void display(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t); // 4 numbers + DP + time to keep display on

volatile uint8_t adcresult, recd_data, count_light, count_dark, count_bits; /* used in adc interrupt but declared global for easy access */
volatile int16_t diff;
volatile uint8_t pinchanges;

int main(void) {
    uint8_t hour = 12, minute = 34, second = 56, day = 01, dow = 0, month = 1;
    uint16_t year = 1970;
    /*init*/
    /*IO for Atmega48:
     * PB3 seg D
     * PB4 seg I/DP
     * 
     * PC0 analog input light sensor
     * PC1 CC1 (Comon Cathode 1)
     * PC2 seg C
     * PC3 CC2
     * PC4 seg B
     * PC5 seg A
     * 
     * PD0 CCDP
     * PD1 seg H
     * PD2 CC3
     * PD3 seg F
     * PD4 seg G
     * PD5 CC4
     * PD6 seg E
     * PD7 lightsensor power
     */
    DDRB = (1 << DDB3) | (1 << DDB4);
    DDRC = (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5);
    DDRD = (1 << DDD0) | (1 << DDD1) | (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6);
    /*power consumption*/
    DIDR0 = (1 << ADC0D); // disable digital input buffer on ADC pin to minimize power consumption
    PORTB = 0xE7; // pullups on unused pins of PORTB.
    ACSR = (1 << ACD); // disable analog comparator
    /*timers*/
    /*RTC timer 2, clock 32.768kHz xtal*/
    ASSR = (1 << AS2); // first set clock to async operation, as that might corrupt registers, and since these are set later it does not matter
    TCCR2A = 0x00; // default / normal mode / no output
    TCCR2B = (1 << CS22) | (1 << CS20); // divide clock by 128, so 32768/128/256 overflow each second.
    TIFR2 = 0x07; // clear interrupt flags
    TIMSK2 = (1 << TOIE2); // enable overflow interrupt
    /*ADC*/
    ADCSRA = 0; // disable/reset ADC
    /*set adc to correct ch and samplerate for autotrigger (adc clock), or disable auto trigger*/
    ADMUX = (1 << ADLAR) | (1 << REFS0); // vcc as adc reference, ADC ch0, left adjust so only upper 8 bits hace to be read
    ADCSRA = (1 << ADIE) | (1 << ADSC) | (1 << ADEN) | (1 << ADPS2); // start ADC, enable interrupt, single conversion, clock /16 (so 1M/16=62.5 kHz);

    sei(); //enable interrupts
    PORTD |= (1 << PORTD7); /*Turn sensor on*/

    //display tests
#if 0
    //blink dp:
    // set common cathodes high (OFF) for all but CCDP
    PORTD |= (1 << PORTD2) | (1 << PORTD5);
    PORTC |= (1 << PORTC1) | (1 << PORTC3);
    for (uint8_t i = 0; i < 4; i++) {
        PORTB |= (1 << PORTB4); // set I/DP on
        waitalongbit();
        PORTB &= ~(1 << PORTB4); // set I/DP OFF
        waitalongbit();
    }
#endif
#if 0
    while (1) { // display and char rom test
        char test[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 'a', 'A', 'b', 'c', 'd', 'e', 'f', 'g', 'G', 'h', 'H', 'i', 'j', 'l', 'm', 'M', 'n', 'N', 'o', 'O', 'p', 'q', 'r', 's', 't', 'T', 'u', 'v', 'w', 'y', 'z'};
        uint8_t dp = 0;
        for (uint8_t i = 0; i<sizeof (test); i++) {
            display(test[i], test[(i + 1) % sizeof (test)], test[(i + 2) % sizeof (test)], test[(i + 3) % sizeof (test)], dp, 45);
            dp = ~dp;
        }
    }
#endif
#if 0
    while (1) { // test/debug light sensor
        ADCSRA |= (1 << ADSC); // start a adc conversion for light sensor. 
        display(adcresult / 1000, adcresult / 100 % 10, adcresult / 10 % 10, adcresult % 10, 0, 50);
    };
#endif 

#if 0 // display test and day / month names test
    do {
        uint8_t dow = 0, day = 1, month = 1;
        uint16_t year = 1987;
        for (dow = 0; dow < 7; dow++) {
            display_date(dow, day, month, year);
            day = (day + 1) % 31;
            month++;
        }
        for (; month < 13; month++) {
            display_date(1, day, month, year);
            day = (day + 1) % 31;
        }
    } while (0); // do-while(0) == test once)
#endif

#if 1 // disable to debug power usage and wake-gesture
    while (!read_time_optical()) {
    } // sync time on reboot
#else
    settime_date(hour, minute, second, day, month, year / 100, year % 100, dow); // h,m,s,d,m,y1,y0,dow                
#endif

    ADCSRA &= ~(1 << ADEN); //disable ADC
    PRR |= (1 << PRADC); // power down ADC
    /*set up pin change interrupt on portc.0 == light sensor. that is PCINT8*/
    PCICR = (1 << PCIE1); // enable PCINT1 (PCINT14:8)
    PCMSK1 = (1 << PCINT8); // mask-allow PCINT8

    /*loop*/
    while (1) {
        set_sleep_mode(SLEEP_MODE_PWR_SAVE); // NOT powerdown, else timer2 stops running//keeping time...
        sleep_mode();
        // when it wakes up here, check if there have been multiple pin changes in the last second. if so display time, else, continue sleeping
        if (pinchanges >= 2) {
            gettime_hms(&hour, &minute, &second);
            display_time(hour, minute);
            gettime_hms(&hour, &minute, &second);
            display_time(hour, minute);
            /* Display 'Time' longer then 'Date' */
            gettime_date(&dow, &day, &month, &year);
            display_date(dow, day, month, year);
        }
    }
    return 0;
}

uint8_t read_time_optical() {
    static uint8_t index, receive_buffer[9], now, succes, crc = 0;

    enum {
        SYNC, READDATA, CHECKCRC
    } state = SYNC;
    succes = 0;
    PORTC |= (1<<PORTC0); // internal pullup on on PC0, to change light sensor sensitivity to be less sensitive to light
    //prepare so dp can be used for debug:
    PORTD = 0xA4; // all segmenst 0, all CC high except CCDP, light sensor ON
    PORTC |= (1 << PORTC1) | (1 << PORTC3); // CC's high (inactive)

    do {
        now = TCNT2; // to get somewhat of a fixed sample rate, use Timer 2 (RTC timer, increments at 256 Hz = overflows each second).
        ADCSRA |= (1 << ADSC); // start a adc conversion.
        while ((uint8_t) (TCNT2 - now) < 4) {
            // wait 4/256 = 1/64 of a second, so 64 Samples/s. (15.625 ms)
        };
        if (adcresult > LDTHRESH) { // debug led to aid in determening display threshold calibration and where to hold the watch during sync
            PORTB |= (1 << PORTB4); // debugled dp on
        } else PORTB &= ~(1 << PORTB4);

        switch (state) {
            default:
                state = SYNC;
            case SYNC:
                crc = 0; // reset crc
                index = 0; // reset index
                if (count_bits > 4) state = READDATA; // once the first few bits are in, start reading
                break;
            case READDATA:
                if (count_bits == 8 * (index + 1)) {
                    receive_buffer[index] = recd_data;
                    index++;
                }
                if (count_bits < 3) state = SYNC; // if count_bits has been reset to 0, there has been a time-out
                if (index >= 9) state = CHECKCRC; // all data (incl. CRC) is in
                break;
            case CHECKCRC:
                //crc is calculated here as not to spend time on it during reception.
                for (uint8_t i = 0; i < 8; i++) {
                    crc = _crc8_ccitt_update(crc, receive_buffer[i]);
                }

                if (crc == receive_buffer[8]) {// if crc matches, set time to new received value, else retry   
                    cli(); // prevent clocktick during setting of clock
                    settime_date(receive_buffer[0], receive_buffer[1], receive_buffer[2],
                            receive_buffer[4], receive_buffer[5], receive_buffer[6], receive_buffer[7], receive_buffer[3]); // h,m,s,d,m,y1,y0,dow
                    sei();
                    succes = 1;
                } else {
                    crc = 0;
                    count_bits = 0;
                    state = READDATA; // wait for sync again after time is set or also on CRC mismatch
                }
                break;
        }
        // once CRC is OK and time is set, return.
    } while (!succes);
    PORTC &= ~(1<<PORTC0); // internal pullup OFF on light sensor pin, to change sensitivity back.
    return succes;
}

void waitabit() {
    uint8_t now = TCNT2;
    while (now == TCNT2); // wait 1 counter tick. (32768Hz/128) = 256 Hz, so 3.9 ms, plus a bit for call overhead
}

void waitalongbit() {
    uint8_t i;
    for (i = 0; i < 250; i++) { // 4ms times 250, about a second, give or take call overhead etc. TODO: Could maybe use clocktick instead?
        waitabit();
    }
}

void displayRaw(uint16_t data, uint8_t where) // data bits 0..8 for segment A..I, I also is DP. Where for which CC (4,3,2,1 or DP 0). 
{
    // set all CC's, clear all segments:
    PORTB &= ~((1 << PORTB3) | (1 << PORTB4));
    PORTC &= ~((1 << PC5) | (1 << PC4) | (1 << PC2));
    PORTC |= (1 << PC3) | (1 << PC1);
    PORTD = 0xA5;

    //what? (bits 0..8) (uses PINx to toggle bit, instead of read, modify, write. It starts from reset, so toggle will set it)
    if (data & 0x01) PINC = (1 << PC5); // seg A, portC5
    if (data & 0x02) PINC = (1 << PC4); // seg B, portC4
    if (data & 0x04) PINC = (1 << PC2); // seg C, portC2
    if (data & 0x08) PINB = (1 << PB3); // seg D, portB3
    if (data & 0x10) PIND = (1 << PD6); // seg E, portD6
    if (data & 0x20) PIND = (1 << PD3); // seg F, portD3
    if (data & 0x40) PIND = (1 << PD4); // seg G, portD4
    if (data & 0x80) PIND = (1 << PD1); // seg H, portD1
    if (data & 0x100)PINB = (1 << PB4); // seg I, portB4

    //where? (Here toggle starts from set and will clear it, so driving common cathode low, as intended)
    if (where == 0)PIND = (1 << PD5); // CC4, portD5
    if (where == 1)PIND = (1 << PD2); // CC3, portd2
    if (where == 2)PINC = (1 << PC3); // CC2, portc3
    if (where == 3)PINC = (1 << PC1); // CC1, portc1
    if (where == 4)PIND = (1 << PD0); // CCDP, portd0 
}

uint16_t numTo7Seg(uint8_t num) {
    switch (num) {
        case 0: return 0x3F;
            break;
        case 1: return 0x06;
            break;
        case 2: return 0x5B;
            break;
        case 3: return 0x4F;
            break;
        case 4: return 0x66;
            break;
        case 5: return 0x6D;
            break;
        case 6: return 0x7D;
            break;
        case 7: return 0x07;
            break;
        case 8: return 0x7F;
            break;
        case 9: return 0x6F;
            break;
            /* Letters for mon, tue, wed, thurs, fri, sat, sun
             *  or ma, di, do, vr, za, zo
             * and Jan Feb Mar Apr May/Mei Jun Jul Aug Sept oct nov dec
             * so needed are abcdefghijlmnoprstuvwyz - no k, no q, no x; 
             * and the rest is extra (capitals and lowercase and l 1 | )
             */
        case 'a': return 0x5F;
            break;
        case 'A': return 0x77;
            break;
        case 'b': return 0x7C;
            break;
        case 'c': return 0x58;
            break;
        case 'd': return 0x5E;
            break;
        case 'e': return 0x79;
            break;
        case 'f': return 0x71;
            break;
        case 'g': return 0x6F;
            break;
        case 'G': return 0x7D;
            break;
        case 'h': return 0x74;
            break;
        case 'H': return 0x76;
            break;
        case 'i': return 0x06;
            break;
        case 'I': return 0x30;
            break;
        case '|': return 0x180;
            break;
        case 'j': return 0x0E;
            break;
        case 'l': return 0x38;
            break;
        case 'm': return 0x154;
            break;
        case 'M': return 0xB7;
            break;
        case 'n': return 0x054;
            break;
        case 'N': return 0x37;
            break;
        case 'o': return 0x5C;
            break;
        case 'O': return 0x3F;
            break;
        case 'p': return 0x73;
            break;
        case 'q': return 0x67;
            break;
        case 'r': return 0x50;
            break;
        case 's': return 0x6D;
            break;
        case 't': return 0x78;
            break;
        case 'T': return 0x181;
            break;
        case 'u':
        case 'v': return 0x3E;
            break;
        case 'w': return 0x13E;
            break;
        case 'y': return 0x66;
            break;
        case 'z': return 0x5B;
            break;
        case ' ': return 0x00;
            break;
        default: return 0x11;
            break;
    }
}

void display(uint8_t d4, uint8_t d3, uint8_t d2, uint8_t d1, uint8_t dp, uint8_t time) {
    uint16_t raw[5];
    raw[0] = numTo7Seg(d4);
    raw[1] = numTo7Seg(d3);
    raw[2] = numTo7Seg(d2);
    raw[3] = numTo7Seg(d1);
    raw[4] = dp ? 0x100 : 0; // if dp is set, light the dots
    for (uint8_t i = 0; i <= time; i++) {
        for (uint8_t j = 0; j < 5; j++) {
            displayRaw(raw[j], j);
            waitabit();
        }
    }
}

void display_time(uint8_t hour, uint8_t minute) {
    display(hour / 10, hour % 10, minute / 10, minute % 10, 1, 80);
}

void display_date(uint8_t d_o_w, uint8_t day, uint8_t month, uint16_t year) // day-of-week, day, month, year
{
#define Dutch // Dutch or English?
    uint8_t dow[2];

    switch (d_o_w) {
#ifdef Dutch
        case 0:
            dow[0] = 'M';
            dow[1] = 'A';
            break;
        case 1:
            dow[0] = 'd';
            dow[1] = '|';
            break;
        case 2:
            dow[0] = 'w';
            dow[1] = 'o';
            break;
        case 3:
            dow[0] = 'd';
            dow[1] = 'o';
            break;
        case 4:
            dow[0] = 'v';
            dow[1] = 'r';
            break;
        case 5:
            dow[0] = 'z';
            dow[1] = 'A';
            break;
        case 6:
            dow[0] = 'z';
            dow[1] = 'o';
            break;
        default:
            dow[0] = 'j';
            dow[1] = 'j';
            break;
#endif
#ifdef English
        case 0:
            dow[0] = 'M';
            dow[1] = 'o';
            break;
        case 1:
            dow[0] = 't';
            dow[1] = 'u';
            break;
        case 2:
            dow[0] = 'W';
            dow[1] = 'e';
            break;
        case 3:
            dow[0] = 't';
            dow[1] = 'h';
            break;
        case 4:
            dow[0] = 'f';
            dow[1] = 'r';
            break;
        case 5:
            dow[0] = 's';
            dow[1] = 'A';
            break;
        case 6:
            dow[0] = 's';
            dow[1] = 'u';
            break;
        default:
            dow[0] = 'j';
            dow[1] = 'j';
            break;
#endif
    }
    display(dow[0], dow[1], day / 10, day % 10, 0, 80);
    switch (month) {
        case 1:
            display('j', 'A', 'n', ' ', 0, 80);
            break;
        case 2:
            display('f', 'e', 'b', 'r', 0, 80);
            break;
        case 3:
            display('M', 'A', 'r', ' ', 0, 80);
            break;
        case 4:
            display('A', 'p', 'r', ' ', 0, 80);
            break;
        case 5:
#ifdef Dutch
            display('M', 'e', 'I', ' ', 0, 80);
#endif
#ifdef English
            display('M', 'a', 'y', ' ', 0, 80);
#endif
            break;
        case 6:
            display('j', 'u', 'n', ' ', 0, 80);
            break;
        case 7:
            display('j', 'u', 'l', ' ', 0, 80);
            break;
        case 8:
            display('A', 'u', 'G', ' ', 0, 80);
            break;
        case 9:
            display('s', 'e', 'p', 't', 0, 80);
            break;
        case 10:
            display('O', 'c', 't', ' ', 0, 80);
            break;
        case 11:
            display('N', 'o', 'v', ' ', 0, 80);
            break;
        case 12:
            display('d', 'e', 'c', ' ', 0, 80);
            break;
        default:
#ifdef Dutch
            display('O', 'e', 'p', 's', 0, 80);
#endif
#ifdef English
            display('O', 'o', 'p', 's', 0, 80);
#endif
            break;
    }
    display(year / 1000 % 10, year / 100 % 10, year / 10 % 10, year % 10, 0, 80);
}

ISR(PCINT1_vect) {
    // Pin change interrupt for light sensor
    pinchanges++;
}

ISR(TIMER2_OVF_vect) {
    /*RTC timing with Timer2. Triggers at 1 Hz.*/
    clocktick(); // one second has passed
    pinchanges = 0; //reset number of pinchanges in the last second
}

ISR(ADC_vect) {
    /* ADC for light sensor 
     */

    diff = adcresult; // use previous result to calculate difference
    adcresult = ADCH; //read adc
    diff -= adcresult;

    /*
     * light sensor used for light/dark detection (display on) and to sync time using PWM
     * '1' is 40 ms black folowed by 80 ms white (1-4 samples of bk, 5-7 samples of white)
     * '0' is 80 ms black folwed by 40 ms white (5-7 samples of bk, 1-4 samples of white)
     * reset to bit 0 on longer then 156 ms white (sampletime is 15.6 ms, so, 10 samples)
     */

    if (adcresult > LDTHRESH) { // thresholding.
        count_dark++;
        count_light = 0;
    } else {
        //on reception of first light, check
        if (count_dark > 1) {
            recd_data = recd_data >> 1; // always shift the buffer.
            count_bits++; // count number of bits shifted in
            if (count_dark < 5) { // 1,2,3,4 samples dark it is a '1'. (5,6,7,+ a '0').
                recd_data = recd_data | 0x80;
            }
        }
        count_dark = 0; //reset counter
        count_light++; // sample count is used for timing.
        if (count_light >= 10) { // when line is idle for too long, reset
            count_light = 0;
            count_bits = 0;
        }
    }
}