#ifndef ENCODER_INCLUDED
#define ENCODER_INCLUDED

typedef struct encoder {
    int pin1;
    int pin2;
} encoder;

encoder encoder_init(int pin1, int pin2);
void encoder_enable_irq(encoder enc, void cb(uint gpio, uint32_t events));

#endif
