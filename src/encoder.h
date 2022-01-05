#ifndef ENCODER_INCLUDED
#define ENCODER_INCLUDED

typedef struct encoder {
    int pin1;
    int pin2;
    uint8_t state;
} encoder;

encoder *encoder_init(int pin1, int pin2);
void encoder_enable_irq(encoder *enc, void cb(uint gpio, uint32_t events));
void encoder_callback_handler(encoder *enc);

#endif
