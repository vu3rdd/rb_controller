#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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

/* // get current vfo frequency (A or B) */
/* int getVFO(char AorB) { */
/*     if (AorB == 'A') { */
/*         printf("ZZFA;"); */
/*     } else if (AorB == 'B') { */
/*         printf("ZZFB;"); */
/*     } else { */
/*         return 1234567; */
/*     } */
/*     char resp[20]; // eg: ZZFA00007137000; */

/*     memset(resp, '\0', 20); */

/*     for (int i = 0; i < 20; i++) { */
/*         resp[i] = getchar(); */
/*         if (resp[i] == ';') { */
/*             resp[i] = '\0'; */
/*             break; */
/*         } */
/*     } */

/*     return atoi(&resp[4]); */
/* } */

// get current vfo frequency (A or B)
int getVFO(char AorB) {
    if (AorB == 'A') {
        printf("ZZFA;");
    } else if (AorB == 'B') {
        printf("ZZFB;");
    }
    char resp[20]; // eg: ZZFA00007137000;
    for (int i = 0; i < 20; i++) {
        resp[i] = getchar();
        if (resp[i] == ';') {
            break;
        }
    }

    char freq[12];
    int j = 4;
    for (int i = 0; i < 12; i++) {
        freq[i] = resp[j];
        if (resp[j] == ';') {
            freq[i] = '\0';
            break;
        }
        j++;
    }

    // at this point, the buffer freq contains current frequency as a string.
    return atoi(freq);
}


/* void setVFO(char AorB, int freq) { */
/* } */

mode getMode(void) {
    printf("ZZMD;"); // try to read the current mode
    char mode[20];

    memset(mode, '\0', 20);

    for (int i = 0; i < 20; i++) {
        mode[i] = getchar();
        if (mode[i] == ';')
            mode[i] = '\0';
            break;
    }

    int m = atoi(&mode[4]);

    if (m > DRM) {
        return INVALIDMODE;
    }

    return m;
}
