#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "mcp23017.h"
#include "radio.h"
#include "config.h"

// get current vfo frequency (A or B)
int getVFO(char AorB) {
    if (AorB == 'A') {
        printf("ZZFA;");
    } else if (AorB == 'B') {
        printf("ZZFB;");
    } else {
        return 1234567;
    }
    char resp[20]; // eg: ZZFA00007137000;

    memset(resp, '\0', 20);

    for (int i = 0; i < 20; i++) {
        resp[i] = getchar();
        if (resp[i] == ';') {
            resp[i] = '\0';
            break;
        }
    }

    return (int)strtol(&resp[4], NULL, 10);
}

int getRXAttenuation(void) {
    printf("RA;");
    char attn_buffer[8];

    memset(attn_buffer, '\0', 8);
    for (int i = 0; i < 8; i++) {
        attn_buffer[i] = getchar();
        if (attn_buffer[i] == ';') {
            attn_buffer[i] = '\0';
            break;
        }
    }

    int attn = strtol(&attn_buffer[2], NULL, 10);
    if (errno != 0) {
        return -1;
    }

    // gain is in the form "XX00", so get rid of the last two digits
    // by dividing the number by 100.
    return attn/100;
}

int getTXDrive(void) {
    printf("PC;");
    char drive_buffer[7]; // PCXXX;

    memset(drive_buffer, '\0', 7);
    for (int i = 0; i < 7; i++) {
        drive_buffer[i] = getchar();
        if (drive_buffer[i] == ';') {
            drive_buffer[i] = '\0';
            break;
        }
    }

    int drive = strtol(&drive_buffer[2], NULL, 10);
    if (errno != 0) {
        return -1;
    }

    return drive;
}

int getStepIndex(void) {
    printf("ZZAC;");
    char step_buffer[8];

    memset(step_buffer, '\0', 8);
    for (int i = 0; i < 8; i++) {
        step_buffer[i] = getchar();
        if (step_buffer[i] == ';') {
            step_buffer[i] = '\0';
            break;
        }
    }

    int step_index = strtol(&step_buffer[4], NULL, 10);
    if (errno != 0) {
        return -1;
    }

    return step_index;
}

void setStepSize(unsigned int index) {
    // valid step indices
    // 0 - 1hz
    // 1 - 10hz
    // 2 - 100hz
    // 3 - 1k
    // 4 - 10k
    // 5 - 100k
    // 6 - 1M
    if (index >= 7) {
        // invalid index, do nothing
        return;
    } else {
        printf("ZZAC%02d;", index);
    }
}

int getAudioGain(void) {
    printf("ZZAG;");
    char audiogain_buf[9];

    memset(audiogain_buf, '\0', 9);
    for (int i = 0; i < 9; i++) {
        audiogain_buf[i] = getchar();
        if (audiogain_buf[i] == ';') {
            audiogain_buf[i] = '\0';
            break;
        }
    }

    int ag = strtol(&audiogain_buf[4], NULL, 10);
    if (errno != 0) {
        return -1;
    }

    return ag;
}

int getMicGain(void) {
    printf("ZZMG;");
    char micgain_buf[9];

    memset(micgain_buf, '\0', 9);
    for (int i = 0; i < 9; i++) {
        micgain_buf[i] = getchar();
        if (micgain_buf[i] == ';') {
            micgain_buf[i] = '\0';
            break;
        }
    }

    int mic_gain = strtol(&micgain_buf[4], NULL, 10);
    if (errno != 0) {
        return -1;
    }

    return mic_gain;
}

mode getMode(void) {
    printf("ZZMD;"); // try to read the current mode
    char mode[20];

    memset(mode, '\0', 20);

    for (int i = 0; i < 20; i++) {
        mode[i] = getchar();
        if (mode[i] == ';') {
            mode[i] = '\0';
            break;
        }
    }

    int m = (int) strtol(&mode[4], NULL, 10);

    if (errno != 0) {
        return INVALIDMODE;
    }

    if (m > DRM) {
        return INVALIDMODE;
    }

    return m;
}

int getNB(void) {
    printf("ZZNA;"); // try to read the current mode
    char nb[7];

    memset(nb, '\0', 7);

    for (int i = 0; i < 7; i++) {
        nb[i] = getchar();
        if (nb[i] == ';') {
            nb[i] = '\0';
            break;
        }
    }

    return (nb[4] == '0' ? 0 : 1);
}

int getNB2(void) {
    printf("ZZNB;"); // try to read the current mode
    char nb[7];

    memset(nb, '\0', 7);

    for (int i = 0; i < 7; i++) {
        nb[i] = getchar();
        if (nb[i] == ';') {
            nb[i] = '\0';
            break;
        }
    }

    return (nb[4] == '0' ? 0 : 1);
}


int getNR(void) {
    printf("ZZNR;"); // try to read the current mode
    char nr[7];

    memset(nr, '\0', 7);

    for (int i = 0; i < 7; i++) {
        nr[i] = getchar();
        if (nr[i] == ';') {
            nr[i] = '\0';
            break;
        }
    }

    return (nr[4] == '0' ? 0 : 1);
}

int getNR2(void) {
    printf("ZZNS;"); // try to read the current mode
    char nr[7];

    memset(nr, '\0', 7);

    for (int i = 0; i < 7; i++) {
        nr[i] = getchar();
        if (nr[i] == ';') {
            nr[i] = '\0';
            break;
        }
    }

    return (nr[4] == '0' ? 0 : 1);
}

int getSNB(void) {
    printf("ZZNN;"); // try to read the current mode
    char snb[7];

    memset(snb, '\0', 7);

    for (int i = 0; i < 7; i++) {
        snb[i] = getchar();
        if (snb[i] == ';') {
            snb[i] = '\0';
            break;
        }
    }

    return (snb[4] == '0' ? 0 : 1);
}

int getANF(void) {
    printf("ZZNT;"); // try to read the current mode
    char anf[7];

    memset(anf, '\0', 7);

    for (int i = 0; i < 7; i++) {
        anf[i] = getchar();
        if (anf[i] == ';') {
            anf[i] = '\0';
            break;
        }
    }

    return (anf[4] == '0' ? 0 : 1);
}

int getAGCMode(void) {
    printf("ZZGT;"); // try to read the current mode
    char agc[7];

    memset(agc, '\0', 7);

    for (int i = 0; i < 7; i++) {
        agc[i] = getchar();
        if (agc[i] == ';') {
            agc[i] = '\0';
            break;
        }
    }

    return strtol(&agc[4], NULL, 10);
}

void tr_on(radio_state *rs) {
    // do not disturb any of the LPF state. Turn only the T/R line to 1.
    write_register_mcp23008(9, rs->lpf | (1U << 7));
}

void tr_off(radio_state *rs) {
    write_register_mcp23008(9, rs->lpf);
}

void switchBPF(radio_state *rs, int f) {
  int lower_cutoffs[] = {
      1650000,
      3250000,
      6570000,
      9500000,
      13800000,
      16700000,
      19800000,
      23400000,
      26700000,
  };

  int higher_cutoffs[] = {
      2050000,
      4100000,
      7400000,
      10700000,
      15300000,
      18800000,
      22400000,
      26100000,
      30100000,
  };

  // midpoint of the bpf
  int cutoffs[] = {
      1800000,
      3500000,
      7000000,
      10000000,
      14000000,
      18000000,
      21000000,
      24000000,
      28000000,
  };

  static uint8_t oldbpf = 0;

  int control_word[] = {
      0x07,     // what we want at the output of ULN2803 is 1000
      0x0b,     // uln output is 0x04 0100 - inv - 1011 - 0x0b
      0x03,     // 0x0c,
      0x0d,     // 0x02,
      0x05,     // 0x0a,
      0x09,     // 0x06,
      0x01,     // 0x0e,
      0x0e,     // 0x01,
      0x06,     // 0x09,
  };

  int lpf_control_word[] = {
      0xf0,     // what we want at the output of ULN2803 is b0000
      0x70,     // uln output is b1000
      0xb0,     // 0x40,
      0x30,     // 0xc0,
      0xd0,     // 0x20,
      0x50,     // 0xa0,
      0x90,     // 0x60,
      0x10,     // 0xe0,
      0xe0,     // 0x10,
  };

  for (size_t i = 0; i < 9; i++) {
      if (f > lower_cutoffs[i] && f < higher_cutoffs[i]) {
	  int ctrl = control_word[i] | lpf_control_word[i];
	  rs->bpf = ctrl;
	  break;
      }
  }

  if (rs->bpf != oldbpf) {
      write_register_mcp23008(9, rs->bpf);
      oldbpf = rs->bpf;
  }
}

// switch LPF based on frequency
void switchLPF(radio_state *rs, int f) {
#ifdef BPF_VU2YYF
    switchBPF(rs, f);
    return;
#else // BPF_VU2YYF
  static uint8_t oldlpf = 0;
  int cutoffs[] = {
      2000000,
      3000000,
      5000000,
      9000000,
      16000000,
      30000000,
  };

  for (size_t i = 0; i < 6; i++) {
      if (f < cutoffs[i]) {
          rs->lpf = 1 << i;
          break;
      }
  }

  if (rs->lpf != oldlpf) {
      write_register_mcp23008(9, rs->lpf | rs->antsel | rs->rxant);
      oldlpf = rs->lpf;
  }
#endif // BPF_VU2YYF
}


radio_state *radio_init(void){
    radio_state *s = (radio_state *)malloc(sizeof(radio_state));

    s->zzac_index = 0; // getStepIndex();
    s->ctune = 0;
    s->zzmd_index = 0;
    s->rx_gain = 0;
    s->tx_gain = 50;
    s->audio_gain = 30;
    s->cw_speed = 20; // default CW speed is 20 WPM
    s->mic_gain = 10;
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
    s->drive_enable = 0;
    s->agc_mode = 0;
    s->zoom_val = 0;
    s->antsel = 0;
    s->rxant = 0;
    s->lpf = 0;
    s->zzmd1_index = 0;
    s->power = true;
    s->mute = false;
    return s;
}
