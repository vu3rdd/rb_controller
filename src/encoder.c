#include "pico/stdlib.h"
#include "encoder.h"

encoder encoder_init(int pin1, int pin2) {
    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1);

    gpio_init(pin2);
    gpio_set_dir(pin2, GPIO_IN);
    gpio_pull_up(pin2);
}

void encoder_enable_irq(encoder enc, void cb(uint gpio, uint32_t events)) {
    gpio_set_irq_enabled_with_callback(
        enc.pin1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, cb);

    gpio_set_irq_enabled_with_callback(
        enc.pin2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, cb);
}
