#include <stdlib.h>

#include "radio.h"

radio_state *radio_init(void){
    radio_state *s = (radio_state *)malloc(sizeof(radio_state));

    s->ctune = 0;
    s->zzmd_index = 0;
    s->rx_gain = 0;
    s->tx_gain = 50;
    s->audio_gain = 25;
    s->zzac_index = 0;
    s->nb_val = 0;
    s->snb_val = 0;
    s->anf_val = 0;
    s->nr_val = 0;
    s->vox_val = 0;
    s->split_val = 0;
    s->lock_val = 0;
    s->filter_val = 0;
    s->rit_val = 0;
    s->rit = false;
    s->zoom_enable = false;
    s->drive_enable = false;
    s->agc_mode = 0;
    s->zoom_val = 0;
    s->f = 0;
    s->antsel = 0;
    s->rxant = 0;
    s->lpf = 0;
    s->zzmd1_index = 0;
    s->power = true;
    s->mute = false;
    return s;
}

