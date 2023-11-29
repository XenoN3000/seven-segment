#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


enum P {
    P0 = 0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7
};

#define UNTOUCHED 255
#define KEYPAD_COLS 3
#define KEYPAD_ROWS 4

#define SEL_PRT PORTB
#define SEL_DDR DDRB


#define SVN_SEG_DDR DDRA
#define SVN_SEG_PRT PORTA

#define KEY_PRT PORTD
#define KEY_PIN PIND
#define KEY_DDR DDRD


void IO_Init();

static inline void keyfind();

void seg7_update(unsigned char);

void stream_to_svg();


unsigned char SEG7_TABLE[11] = {0xBF, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0};
unsigned char SEG7[8] = {0, 0, 0, 0, 0, 0, 0, 0};

                            //10000000       000000001         00000010
unsigned char SEG7_EN[8] = {(1 << P7), (1 << P0), (1 << P1), (1 << P2), (1 << P3), (1 << P4), (1 << P5),
                            (1 << P6)};
unsigned char INDEX = 0;


char keypad[4][3] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9},
        {8, 0, 0}
};


unsigned char colloc, rowloc;


static inline void keyfind() {
    while (1) {
        KEY_DDR = 0xF0;
        KEY_PRT = 0xFF;

        do {
            KEY_PRT &= 0x0F;
            asm("NOP");
            colloc = (KEY_PIN & 0x0F);
        } while (colloc != 0x0F);

        do {
            do {
                _delay_ms(20);
                colloc = (KEY_PIN & 0x0F);
            } while (colloc == 0x0F);

            _delay_ms(40);
            colloc = (KEY_PIN & 0x0F);
        } while (colloc == 0x0F);

        KEY_PRT = 0xEF;
        asm("NOP");
        colloc = (KEY_PIN & 0x0F);
        if (colloc != 0x0F) {
            rowloc = 0;
            break;
        }

        KEY_PRT = 0xDF;
        asm("NOP");
        colloc = (KEY_PIN & 0x0F);
        if (colloc != 0x0F) {
            rowloc = 1;
            break;
        }

        KEY_PRT = 0xBF;
        asm("NOP");
        colloc = (KEY_PIN & 0x0F);
        if (colloc != 0x0F) {
            rowloc = 2;
            break;
        }

        KEY_PRT = 0x7F;
        asm("NOP");
        colloc = (KEY_PIN & 0x0F);
        if (colloc != 0x0F) {
            rowloc = 3;
            break;
        }
    }

    if (colloc == 0x0E)
        seg7_update(keypad[rowloc][0]);
    else if (colloc == 0x0D)
        seg7_update(keypad[rowloc][1]);
    else if (colloc == 0x0B)
        seg7_update(keypad[rowloc][2]);
    else
        return;
}


void seg7_update(unsigned char c) {
    SEG7[INDEX] = c;
    INDEX++;
    if (INDEX > 7) INDEX = 0;
}


void stream_to_svg() {

    for (int i = 0; i < 8; ++i) {

        SEL_PRT = SEG7_EN[i];
        SVN_SEG_PRT = SEG7_TABLE[SEG7[i]];
        _delay_ms(1);
    }

}


void IO_Init() {

    SVN_SEG_DDR = (1 << P0) | (1 << P1) | (1 << P2) | (1 << P3) | (1 << P4) | (1 << P5) | (1 << P6) |
                  (1 << P7); //11111111

    SVN_SEG_PRT = (0 << P0) | (0 << P1) | (0 << P2) | (0 << P3) | (0 << P4) | (0 << P5) | (0 << P6) |
                  (0 << P7);

    SEL_DDR = (1 << P0) | (1 << P1) | (1 << P2) | (1 << P3) | (1 << P4) | (1 << P5) | (1 << P6) |
              (1 << P7);

    SEL_PRT = (0 << P0) | (0 << P1) | (0 << P2) | (0 << P3) | (0 << P4) | (0 << P5) | (0 << P6) |
              (0 << P7);

    KEY_DDR = (0 << P0) | (0 << P1) | (0 << P2) | (0 << P3) | (1 << P4) | (1 << P5) | (1 << P6) |
              (1 << P7); //00001111

    KEY_PRT = (1 << P0) | (1 << P1) | (1 << P2) | (1 << P3) | (1 << P4) | (1 << P5) | (1 << P6) |
              (1 << P7);


    TIMSK = (1 << TOIE0);
    TCCR0 = (0 << WGM00) | (0 << COM01) | (0 << COM00) | (0 << WGM01) | (1 << CS02) | (0 << CS01) | (0 << CS00);
    TCNT0 = 255;
    OCR0 = 0x00;

    _BV(8); // 00000001 &   PORTA  = 000000000;


}


ISR(TIMER0_OVF_vect) {
    stream_to_svg();
    TCNT0 = 255;
}

int main() {
    IO_Init();

    sei();

    while (1) {
        keyfind();
    };

    return 0;
}
