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

typedef enum mode {
    INVALIDMODE = -1,
    LSB  = 0,
    USB  = 1,
    DSB  = 2,
    CWL  = 3,
    CWU  = 4,
    FM   = 5,
    AM   = 6,
    DIGU = 7,
    SPEC = 8,
    DIGL = 9,
    SAM  = 10,
    DRM  = 11,
} mode;

// exposed functions
radio_state *radio_init(void);

// these functions directly read from the radio and not from the
// cached state type
int getVFO(char AorB);
mode getMode(void);

#endif
