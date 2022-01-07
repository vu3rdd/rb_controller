#include "encoder.h"
#include "pins.h"

#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>

// https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
// GPLv3

const unsigned char ttable[8][4] = {
    { R_CW_NEXT,   R_CW_BEGIN,   R_CW_FINAL,   R_START },            // R_CW_NEXT (00)
    { R_CW_NEXT,   R_START,      R_CW_FINAL,   R_START | DIR_CW },   // R_CW_FINAL (01)
    { R_CW_NEXT,   R_CW_BEGIN,   R_START,      R_START },            // R_CW_BEGIN (10)
    { R_START,     R_CW_BEGIN,   R_CCW_BEGIN,  R_START },            // R_START (11)

    { R_CCW_NEXT,  R_CCW_FINAL,  R_CCW_BEGIN,  R_START },            // R_CCW_NEXT
    { R_CCW_NEXT,  R_CCW_FINAL,  R_START,      R_START | DIR_CCW },  // R_CCW_FINAL
    { R_CCW_NEXT,  R_START,      R_CCW_BEGIN,  R_START },            // R_CCW_BEGIN
    { R_START,     R_START,      R_START,      R_START },            // ILLEGAL
};

encoder *encoder_init(int pin1, int pin2) {
    encoder *enc = malloc(sizeof(encoder));

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1);

    gpio_init(pin2);
    gpio_set_dir(pin2, GPIO_IN);
    gpio_pull_up(pin2);

    enc->pin1 = pin1;
    enc->pin2 = pin2;
    enc->state = 3;
    enc->count = 0;

    return enc;
}

void encoder_isr(encoder *enc) {
    unsigned char pin_state = (gpio_get(enc->pin2) << 1) | gpio_get(enc->pin1);
    unsigned char state = ttable[enc->state & 0x07][pin_state];

    if (state & DIR_CW) {
        enc->count++;
    }
    if (state & DIR_CCW) {
        enc->count--;
    }

    enc->state = state;
}
