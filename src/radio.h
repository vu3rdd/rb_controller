#ifndef RADIO_INCLUDE
#define RADIO_INCLUDE

#include "pico/stdlib.h"

#include <stdbool.h>

typedef struct radio_state {
    int ctune;
    int zzmd_index;
    int rx_gain;
    int tx_gain;
    int audio_gain;
    int zzac_index;
    int nb_val;
    int snb_val;
    int anf_val;
    int nr_val;
    int vox_val;
    int split_val;
    int lock_val;
    int filter_val;
    int rit_val;
    bool rit;
    bool zoom_enable;
    bool drive_enable;
    int agc_mode;
    int zoom_val;
    long long f;
    uint8_t antsel;
    uint8_t rxant;
    uint8_t lpf;
    int zzmd1_index;
    bool power;
    bool mute;
} radio_state;

// exposed functions
radio_state *radio_init(void);

#endif
