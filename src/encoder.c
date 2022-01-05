#include "pico/stdlib.h"
#include "encoder.h"

#include <stdlib.h>

#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#define R_START 0x0

//#define HALF_STEP

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
    // R_START (00)
    {R_START_M, R_CW_BEGIN, R_CCW_BEGIN, R_START},
    // R_CCW_BEGIN
    {R_START_M | DIR_CCW, R_START, R_CCW_BEGIN, R_START},
    // R_CW_BEGIN
    {R_START_M | DIR_CW, R_CW_BEGIN, R_START, R_START},
    // R_START_M (11)
    {R_START_M, R_CCW_BEGIN_M, R_CW_BEGIN_M, R_START},
    // R_CW_BEGIN_M
    {R_START_M, R_START_M, R_CW_BEGIN_M, R_START | DIR_CW},
    // R_CCW_BEGIN_M
    {R_START_M, R_CCW_BEGIN_M, R_START_M, R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const unsigned char ttable[7][4] = {
    // R_START
    {R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START},
    // R_CW_FINAL
    {R_CW_NEXT, R_START, R_CW_FINAL, R_START | DIR_CW},
    // R_CW_BEGIN
    {R_CW_NEXT, R_CW_BEGIN, R_START, R_START},
    // R_CW_NEXT
    {R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START},
    // R_CCW_BEGIN
    {R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START},
    // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | DIR_CCW},
    // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

encoder *encoder_init(int pin1, int pin2) {
    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1);

    gpio_init(pin2);
    gpio_set_dir(pin2, GPIO_IN);
    gpio_pull_up(pin2);

    encoder *enc = malloc(sizeof(encoder));
    enc->pin1 = pin1;
    enc->pin2 = pin2;
    enc->state = 0;
}

void encoder_enable_irq(encoder *enc, void cb(uint gpio, uint32_t events)) {
    gpio_set_irq_enabled_with_callback(
        enc->pin1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, cb);

    gpio_set_irq_enabled_with_callback(
        enc->pin2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, cb);
}

void encoder_callback_handler(encoder *enc) {
    unsigned char pinstate = (gpio_get(enc->pin2) << 1) | gpio_get(enc->pin1);

    // Determine new state from the pins and state table.
    enc->state = ttable[enc->state & 0xf][pinstate];

    // Return emit bits, ie the generated event
    pinstate = enc->state & 0x30;

    if (pinstate) {
        enc->state = (int)pinstate;
  }
}
