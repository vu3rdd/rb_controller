#ifndef ENCODER_INCLUDED
#define ENCODER_INCLUDED

typedef struct encoder {
    volatile unsigned char pin1;
    volatile unsigned char pin2;
    volatile unsigned int state;
    unsigned int count;
} encoder;

// algorithm from Ben Buxton
// https://github.com/buxtronix/arduino/tree/master/libraries/Rotary

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */

// Use the full-step state table (emits a code at 00 only)
// CW: 0,0 -> 1,0 -> 0,1 -> 0,0
// CCW: 0,0 -> 0,1 -> 1,0 -> 0,0

// EC11E (AB): 10 -> 11 -> 01 -> 00 -> ...
// i.e. 2 -> 3 -> 1 -> 0
// if (BA): 01 -> 11 -> 10 -> 00. i.e. 0 -> 1 -> 3 -> 2 -> 0

#define DIR_NONE 0x0
#define DIR_CW   0x10  // Clockwise step.
#define DIR_CCW  0x20  // Counter-clockwise step.

#define R_START     0x3
#define R_CW_BEGIN  0x2
#define R_CW_NEXT   0x0
#define R_CW_FINAL  0x1

#define R_CCW_BEGIN 0x6
#define R_CCW_NEXT  0x4
#define R_CCW_FINAL 0x5


encoder *encoder_init(int, int);
void encoder_isr(encoder *enc);

#endif
