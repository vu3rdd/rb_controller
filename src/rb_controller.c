/**************************************
 * piHPSDR Controller for Radioberry
 * firmware V1.0
 * DE VU2DLE Dileep
 **************************************/
#include "hardware/i2c.h"

#include "mcp23017.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radio.h"
#include "encoder.h"
#include "pins.h"

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
#endif


int rows[] = { KPR0, KPR1, KPR2, KPR3 };
int cols[] = { KPC0, KPC1, KPC2, KPC3 };

encoder *encoders[5];

unsigned long time = 0;
const int delayTime = 50; // Delay for every push button may vary

bool keyer_control = false;
uint32_t int_status;

void waitforradio();

volatile int KeyPressed = false, LongKeyPressed = false, Keyval = 0, old_Keyval;
unsigned int col;
volatile uint kp_gpio = KPCX;

unsigned int keypad_4X4[4][4] = {
    { 1,   2,  3,  4  },
    { 5,   6,  7,  8  },
    { 9,  10, 11,  12 },
    { 13, 14, 15,  16 }
};

// buttons mapping (name to an integer defined in keypad_4x4

// top left
#define BTN_ON_OFF      13
#define BTN_ANT_SEL      9
#define BTN_VOX          5

// bottom left
#define BTN_BAND_DWN     1
#define BTN_BAND_UP      2

// bottom middle
#define BTN_LSB_USB_AM   3
#define BTN_LCW_UCW_DIG  4

// top right 3x3
#define BTN_VFO_A_B      7
#define BTN_MHZ_STEP    10
#define BTN_VFO_SWAP     6
#define BTN_SPLIT        8
#define BTN_NB          11
#define BTN_NR          12
#define BTN_AGC         14
#define BTN_CTUNE       15
#define BTN_FLOCK       16

// bottom right (fed via MCP23008)
#define BTN_FSTEP      191
#define BTN_RXANT      223

#define PTT_OUT_RELAY        1
#define POWER_ON_RELAY       2
#define BIAS_OUT            32
#define TR_RELAY_OUT        64
#define AM_AMP_MUTE_ON_PTT 128

bool MHZ_enable = false;
uint8_t MCP23017_GPIOA_val = 0;

char zzmd_val[3][3]  = { "00", "01", "06" };
char zzmd1_val[3][3] = { "03", "04", "07" };

// I2C reserves some addresses for special purposes. We exclude these from the
// scan. These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Pins
const uint sda_pin = 8;
const uint scl_pin = 9;

static const bool MIRROR_INTERRUPTS =
    true; // save a gpio by mirroring interrupts across both banks
static const bool OPEN_DRAIN_INTERRUPT_ACTIVE = false;
static const bool POLARITY_INTERRUPT_ACTIVE_LOW = false;
static const int MCP_PINS_INPUT = 0xff00;
static const int MCP_PINS_PULL_UP = 0xff00;
static const int MCP_ALL_PINS_COMPARE_TO_LAST = 0x0000;
static const int MCP_ALL_PINS_INTERRUPT_ENABLED = 0xff00;
bool interrupt_on_mcp0 = false;
#define MCP23017_GPIOA 0x12 // Port Register - write modifies latch
#define MCP23017_GPIOB 0x13 // Port Register - write modifies latch

/*
I2C Bus Scan
   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
00 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
10 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
20 .  @  .  .  .  .  @  .  .  .  .  .  .  .  .  .
30 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
40 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
50 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
60 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
70 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
*/

const uint i2c_8bit_addr = 0x26;
const uint i2c_16bit_addr = 0x21;
#define MCP_IRQ_GPIO_PIN 21

void mcp1_gpio_callback(uint gpio, uint32_t events) {
  printf("IRQ callback on GPIO %u, events: %lu\n", gpio, events);
  if (gpio == MCP_IRQ_GPIO_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
    interrupt_on_mcp0 = true;
  }
}

void setup_input(uint gpio_irq) {
  int result = 0;

  result = setup(MIRROR_INTERRUPTS, POLARITY_INTERRUPT_ACTIVE_LOW);
  result = set_io_direction(MCP_PINS_INPUT);
  result = set_pullup(MCP_PINS_PULL_UP);
  result = set_interrupt_type(MCP_ALL_PINS_COMPARE_TO_LAST);
  // result = enable_interrupt(MCP_ALL_PINS_INTERRUPT_ENABLED);

  // gpio_set_irq_enabled_with_callback(gpio_irq, GPIO_IRQ_EDGE_FALL, true,
  //                                   &mcp1_gpio_callback);

  // once we are listening for interrupts clear previous ones just incase
  // int int_values = get_interrupt_values();
}

void rit_enc_handler(radio_state *rs, encoder *ritenc) {
  static unsigned int rit_last_count;
  int rit_increment = 10;

  if (ritenc->count != rit_last_count) {
      if (ritenc->count > rit_last_count) {
          printf("ZZRD;");
      } else {
          printf("ZZRU;");
      }

      rit_last_count = ritenc->count;
  }
}

// generic encoder that can set CW speed, mic gain
// single press to toggle to tx drive level
void rxgain_enc_handler(radio_state *rs, encoder *rxgainenc) {
  static unsigned int rxgain_last_count;

  switch (rs->drive_enable) {
  case 0: {
      // rx gain (rather attenuation) range is (-12, +48).
      // this is mapped to a number (integer) between 0 and 99 internally
      // that number is what is returned by the command when asked
      // for the attenuation.
      if (rxgainenc->count != rxgain_last_count) {
          int current_gain = getRXAttenuation();
          rs->rx_gain = current_gain;

          // Take action here
          if (rxgainenc->count > rxgain_last_count) {
              if (rs->rx_gain < 60)
                  rs->rx_gain++;
          } else if (rxgainenc->count < rxgain_last_count) {
              if (rs->rx_gain > 0)
                  rs->rx_gain--;
          }
          printf("RA%02d;", rs->rx_gain);
      }
      break;
  }
  case 1:
      // tx gain
      if (rxgainenc->count != rxgain_last_count) {
          // Take action here
          rs->tx_gain = getTXDrive();

          if (rxgainenc->count < rxgain_last_count) {
              rs->tx_gain++;
              if (rs->tx_gain >= 100)
                  rs->tx_gain = 100;
          } else if (rxgainenc->count > rxgain_last_count) {
              if (rs->tx_gain > 0)
                  rs->tx_gain--;
          }
          printf("PC%03d;", rs->tx_gain);
      }
      break;
  case 2: {
      // cw speed or mic gain depending on mode
      // increment by one.
      // valid values are 0 to 60, so there should be a cap
      if (rxgainenc->count != rxgain_last_count) {
          int mode = getMode();
          if (mode == CWL || mode == CWU) {
              if (rxgainenc->count < rxgain_last_count) {
                  rs->cw_speed++;
                  rs->cw_speed = (rs->cw_speed > 60 ? 60 : rs->cw_speed);
              } else if (rxgainenc->count > rxgain_last_count) {
                  rs->cw_speed--;
                  rs->cw_speed = (rs->cw_speed < 0 ? 0 : rs->cw_speed);
              }
              printf("ZZCS%02d;", rs->cw_speed);
          } else if (mode == LSB || mode == USB || mode == AM || mode == DSB) {
              rs->mic_gain = getMicGain();
              if (rxgainenc->count < rxgain_last_count) {
                  // increase mic gain
                  rs->mic_gain++;
                  if (rs->mic_gain > 100) {
                      rs->mic_gain = 100;
                  }
              } else if (rxgainenc->count > rxgain_last_count) {
                  // decrease mic gain
                  rs->mic_gain--;
                  if (rs->mic_gain < 0) {
                      rs->mic_gain = 0;
                  }
              }
              // send the command
              printf("ZZMG%03d;", rs->mic_gain);
          }
      }
      break;
  }
  default:
      break;
  }

  rxgain_last_count = rxgainenc->count;
}

void writemcp23017() {
    write_register(MCP23017_GPIOA, ~MCP23017_GPIOA_val);
}

/* // timer tick */
bool timer_ticked = false;
bool repeating_timer_callback(struct repeating_timer *t) {
    timer_ticked = true;

    return true;
}

void vfo_enc_handler(radio_state *rs, encoder *vfo_enc) {
  static unsigned int vfo_last_count;

  // note: this is a high pulse-per-rotation encoder. It emits 4 steps
  // for each full-pulse period. Also, since the pulse rate is high,
  // it may be better to divide this rate to something smaller to make
  // the turning a lot smoother.

  if (vfo_enc->count != vfo_last_count) {
    // Take action here
    if (vfo_enc->count < vfo_last_count) {
      printf("ZZAF01;");
    }
    if (vfo_enc->count > vfo_last_count) {
      printf("ZZAE01;");
    }
    // sleep_ms(10);
    int f = getVFO('A');
    switchLPF(rs, f);
  }

  vfo_last_count = vfo_enc->count;
}

void audio_gain_enc_handler(radio_state *rs, encoder *audio_gain_enc) {
  static unsigned int audio_gain_last_count;

  if (audio_gain_enc->count != audio_gain_last_count) {
      // Take action here
      // get audio gain
      rs->audio_gain = getAudioGain();
      if (audio_gain_enc->count < audio_gain_last_count) {
              rs->audio_gain++;
              if (rs->audio_gain > 100) {
                  rs->audio_gain = 100;
              }
      } else if (audio_gain_enc->count > audio_gain_last_count) {
              rs->audio_gain--;
              if (rs->audio_gain < 0) {
                  rs->audio_gain = 0;
              }
      }
      printf("ZZAG%03d;", rs->audio_gain);
  }

  audio_gain_last_count = audio_gain_enc->count;
}

void filter_enc_handler(radio_state *rs, encoder *filter_enc) { // Zoom - Filter
  static unsigned int filter_last_count;

  if (filter_enc->count != filter_last_count) {
    if (!rs->zoom_enable) {
      // Take action here
      if (filter_enc->count < filter_last_count) {
        rs->filter_val += 1;
        if (rs->filter_val >= 8)
          rs->filter_val = 8;
      } else {
        if (rs->filter_val > 0)
          rs->filter_val--;
        else
          rs->filter_val = 0;
      }
      printf("ZZFI%02d;", rs->filter_val);
      // printf("FW%04d;", filter_val);
    } else {
      if (filter_enc->count > filter_last_count) {
        rs->zoom_val++;
        if (rs->zoom_val > 8)
          rs->zoom_val = 8;
      } else {
        if (rs->zoom_val <= 1)
          rs->zoom_val = 1;
        else
          rs->zoom_val--;
      }
      printf("ZZPY%03d;", rs->zoom_val);
    }
  }

  filter_last_count = filter_enc->count;
}

void keypad_Handler(radio_state *rs) {
    for (int i = 0; i < 4; i++) {
        gpio_put(rows[i], 0);
    }

    if (kp_gpio == KPC0)
        col = 0;
    else if (kp_gpio == KPC1)
        col = 1;
    else if (kp_gpio == KPC2)
        col = 2;
    else if (kp_gpio == KPC3)
        col = 3;

    // printf("Keypad_handler: KeyVal = %d\n", Keyval);
    for (unsigned int row = 0; (row < 4); row++) {
        gpio_put(rows[row], 1);
        sleep_ms(10);
        if (gpio_get(kp_gpio)) {
            Keyval = keypad_4X4[row][col];
            KeyPressed = true;
            sleep_ms(250);
            if (gpio_get(kp_gpio)) {
                LongKeyPressed = true;
            }
        }
        gpio_put(rows[row], 0);
    }

    for (int i = 0; i < 4; i++) {
        gpio_put(rows[i], 1);
    }

    if (KeyPressed) {
        switch (Keyval) {
        case BTN_CTUNE:
            printf("ZZCN%d;", rs->ctune);
            if (rs->ctune == 0)
                rs->ctune = 1;
            else
                rs->ctune = 0;
            break;
        case BTN_LSB_USB_AM:
            printf("ZZMD%s;", zzmd_val[rs->zzmd_index]);
            rs->zzmd_index++;
            if (rs->zzmd_index == 3)
                rs->zzmd_index = 0;
            break;
        case BTN_LCW_UCW_DIG:
            printf("ZZMD%s;", zzmd1_val[rs->zzmd1_index]);
            rs->zzmd1_index++;
            if (rs->zzmd1_index == 3)
                rs->zzmd1_index = 0;
            break;
        case BTN_AGC: {
            rs->agc_mode = getAGCMode();
            switch (rs->agc_mode) {
            case 0:
                rs->agc_mode = 2;
                break;
            case 2:
                rs->agc_mode = 3;
                break;
            case 3:
                rs->agc_mode = 4;
                break;
            case 4:
                rs->agc_mode = 2;
                break;
            default:
                rs->agc_mode = 2;
            }
            printf("ZZGT%d;", rs->agc_mode);
            break;
        }
        case BTN_BAND_UP: {
            int f = getVFO('A');
            if (!MHZ_enable) {
                printf("ZZBU;");
                sleep_ms(15);
                // update f
                f = getVFO('A');
            } else {
                f += 1e6;
                if (f > 30e6)
                    f = 30e6;
                printf("ZZFA%011lld;", f);
                // sleep_ms(15);
            }
            switchLPF(rs, f);
            break;
        }
        case BTN_BAND_DWN: {
            int f = getVFO('A');
            if (!MHZ_enable) {
                printf("ZZBD;");
                // sleep_ms(15);
                f = getVFO('A');
            } else {
                if (f > 1e6)
                    f -= 1e6;
                printf("ZZFA%011lld;", f);
                sleep_ms(15);
            }
            switchLPF(rs, f);
            break;
        }
        case BTN_NB: {
            int nb_val = getNB();
            int nb2_val = getNB2();

            if (nb_val == 0 && nb2_val == 0) {
                // nb and nb2 is off, so turn on nb
                nb_val = 1;
            } else if (nb_val == 1 && nb2_val == 0) {
                // nb is on, nb2 is off, so turn off nb and turn on nb2
                nb_val = 0;
                nb2_val = 1;
            } else if (nb_val == 0 && nb2_val == 1) {
                // nb is off, nb2 is on -> turn off nb and nb2
                nb_val = 0;
                nb2_val = 0;
            }

            if (LongKeyPressed) {
                rs->snb_val = getSNB();
                rs->snb_val = (rs->snb_val + 1) % 2;
                printf("ZZNN%d;", rs->snb_val);
                break;
            }
            printf("ZZNA%d;ZZNB%d;", nb_val, nb2_val);
            break;
        }
        case BTN_NR: {
            int nr_val = getNR();
            int nr2_val = getNR2();

            if (nr_val == 0 && nr2_val == 0) {
                // nr and nr2 is off, so turn on nb
                nr_val = 1;
            } else if (nr_val == 1 && nr2_val == 0) {
                // nr is on, nr2 is off, so turn off nr and turn on nr2
                nr_val = 0;
                nr2_val = 1;
            } else if (nr_val == 0 && nr2_val == 1) {
                // nr is off, nr2 is on -> turn off nr and nr2
                nr_val = 0;
                nr2_val = 0;
            }

            if (LongKeyPressed) {
                rs->anf_val = getANF();
                // toggle ANF state
                rs->anf_val = (rs->anf_val + 1) % 2;
                printf("ZZNT%d;", rs->anf_val);
                break;
            }

            printf("ZZNR%d;ZZNS%d;", nr_val, nr2_val);
            break;
        }
        case BTN_VFO_A_B:
            printf("ZZVS0;");
            break;
        case BTN_MHZ_STEP:
            if (MHZ_enable) {
                MHZ_enable = false;
                // printf("ZZTI0;");
            } else {
                MHZ_enable = true;
                // printf("ZZTI1;");
            }
            // printf("ZZVS1;");
            break;
        case BTN_VFO_SWAP: {
            printf("ZZVS2;");

            int f = getVFO('A');
            switchLPF(rs, f);
            break;
        }
        case BTN_VOX:
            if (rs->vox_val == 0)
                rs->vox_val = 1;
            else
                rs->vox_val = 0;
            printf("VX%d;", rs->vox_val);
            break;
        case BTN_SPLIT: {
            // first read the current mode. If the mode is CW, then VFOB=VFOA+1kc and SPLIT ON
            // if mode is SSB (LSB/USB), then VFOB = vfoA + 5kc and SPLIT ON
            int mode = getMode();

            if (mode == CWL || mode == CWU) {
                // check if mode is CW
                // vfoB = vfoA + 1khz
                // first read vfo A
                int vfoA = getVFO('A');

                // calculate the new vfo B value
                int vfoB_int = vfoA + 1000;

                // set vfo B value
                char vfoB[17];
                sprintf(vfoB, "ZZFB%011d;", vfoB_int);
                printf("%s", vfoB);
            } else if (mode == LSB || mode == USB) {
                // we are in LSB or USB
                // vfoB = vfoA + 5khz
                // first read vfo A
                int vfoA = getVFO('A');

                // calculate the new vfo B value
                int vfoB_int = vfoA + 5000;

                // set vfo B value
                char vfoB[17];
                sprintf(vfoB, "ZZFB%011d;", vfoB_int);
                printf("%s", vfoB);
            }

            if (rs->split_val == 0)
                rs->split_val = 1;
            else
                rs->split_val = 0;
            printf("ZZSP%d;", rs->split_val);
            break;
        }
        case BTN_FLOCK:
            if (rs->lock_val == 0)
                rs->lock_val = 1;
            else
                rs->lock_val = 0;
            printf("ZZVL%d;", rs->lock_val);
            break;
        case BTN_ANT_SEL:
            if (rs->antsel == 0)
                rs->antsel = 64;
            else
                rs->antsel = 0;
            write_register(MCP23017_GPIOA, ((rs->antsel == 1) ? 4 : 8));
            write_register_mcp23008(9, rs->lpf | rs->antsel | rs->rxant);
            break;
        case BTN_ON_OFF:
            if (rs->power) {
                printf("#S;");
                sleep_ms(10000);
                MCP23017_GPIOA_val |= POWER_ON_RELAY;
                writemcp23017();
                sleep_ms(20);
                rs->power = false;
            } else {
                MCP23017_GPIOA_val &= ~POWER_ON_RELAY;
                writemcp23017();
                sleep_ms(20);
                waitforradio();
                rs->power = true;
            }
            break;
        default:
            break;
        }
    }
    Keyval = 0;
    KeyPressed = false;
    LongKeyPressed = false;
    kp_gpio = KPCX;
}

void waitforradio(radio_state *rs) {
  // return;
  if (1 == 1) {
    for (int x = 0; x < 30; x++) {
      gpio_put(LED_PIN, 1);
      sleep_ms(500);
      gpio_put(LED_PIN, 0);
      sleep_ms(500);
    }
  }
}

void Keypad4X4_callback_Handler(uint gpio) {
    // printf("Keypad4X4_callback_Handler: %d\n", gpio);
    kp_gpio = gpio;
}

void gpio_callback(uint gpio, uint32_t events) {

    int_status = save_and_disable_interrupts();
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
        time = to_ms_since_boot(get_absolute_time());
        if ((gpio == KPC0) ||
            (gpio == KPC1) ||
            (gpio == KPC2) ||
            (gpio == KPC3)) {
            Keypad4X4_callback_Handler(gpio);
        }
        // printf("gpio_callback gpio = %d\n",gpio);
    }

    if ((gpio == AUDIO_GAIN_ENC_A) || (gpio == AUDIO_GAIN_ENC_B)) {
        encoder_isr(encoders[0]);
    } else if ((gpio == RXGAIN_ENC_A) || (gpio == RXGAIN_ENC_B)) {
        encoder_isr(encoders[1]);
    } else if ((gpio == VFO_ENC_A) || (gpio == VFO_ENC_B)) {
        encoder_isr(encoders[2]);
    } else if ((gpio == RIT_ENC_A) || (gpio == RIT_ENC_B)) {
        encoder_isr(encoders[3]);
    } else if ((gpio == FILTER_ENC_A) || (gpio == FILTER_ENC_B)) {
        encoder_isr(encoders[4]);
    }
    restore_interrupts(int_status);
}

void i2c_expander_handler(radio_state *rs) {
    static int oldpin = -1;
    interrupt_on_mcp0 = false;
    // int pin = get_last_interrupt_pin();
    // int int_values = get_interrupt_values();
    int input_values_1; // = update_input_values();
    input_values_1 = read_register(MCP23017_GPIOB);
    bool long_press = false;

    if (input_values_1 != oldpin) {
        oldpin = input_values_1;

        sleep_ms(250);
        // read again to see if we get the same values
        int input_values_2 = read_register(MCP23017_GPIOB);

        if (input_values_2 == input_values_1) {
            // long press
            long_press = true;
        }

        // printf("InputOK:%d\n", input_values_1);
        switch (input_values_1) {
        case ENC_RIT_SW: {
            if (long_press) {
                printf("ZZRC;");
            } else {
                rs->rit = !rs->rit;
                if (rs->rit) {
                    printf("ZZRT1;");
                } else {
                    printf("ZZRT0;");
                }
            }
            break;
        }
        case ENC_ZOOM_SW: {
            rs->zoom_enable = !rs->zoom_enable;
            break;
        }
        case ENC_MUTE_DRIVE_SW: {
            rs->mute = !rs->mute;
            if (rs->mute) {
                printf("ZZMA1;");
            } else {
                printf("ZZMA0;");
            }
            break;
        }
        case ENC_RX_RFGAIN_SW: {
            // toggle between 0, 1 and 2
            // 0 == rx gain
            // 1 == tx drive
            // 2 == cw speed / mic gain (depending on mode)
            rs->drive_enable = (rs->drive_enable + 1) % 3;
            break;
        }
        case BTN_FSTEP: {
            rs->zzac_index = getStepIndex();
            // cycle through 1, 10, 100, 1k, 10k, 100k, 1M hz
            rs->zzac_index = (rs->zzac_index + 1) % 7;
            printf("ZZAC%02d;", rs->zzac_index);
            break;
        }
        case BTN_RXANT: {
            if (rs->rxant == 0)
                rs->rxant = 128;
            else
                rs->rxant = 0;
            write_register_mcp23008(9, rs->lpf | rs->antsel | rs->rxant);
            break;
        }
        }
    }
}

void ptt_handler(void) {
    static int old_ptt = -1;
    int ptt = gpio_get(PTT_IN);

    if (old_ptt != ptt) {
        old_ptt = ptt;
        if (ptt == 0) { // PTT Pressed
            MCP23017_GPIOA_val &= ~PTT_OUT_RELAY;
            MCP23017_GPIOA_val &= ~TR_RELAY_OUT;
            MCP23017_GPIOA_val |= AM_AMP_MUTE_ON_PTT;

            writemcp23017();
            sleep_ms(20);

            MCP23017_GPIOA_val &= ~BIAS_OUT;

            writemcp23017();
            printf("TX;");
        } else { // PTT released
            // wait a bit before releasing ptt and putting radio into
            // rx mode
            sleep_ms(300);

            printf("RX;");
            MCP23017_GPIOA_val |= BIAS_OUT;

            writemcp23017();
            sleep_ms(20);

            MCP23017_GPIOA_val &= ~AM_AMP_MUTE_ON_PTT;
            MCP23017_GPIOA_val |= PTT_OUT_RELAY;
            MCP23017_GPIOA_val |= TR_RELAY_OUT;

            writemcp23017();
        }
    }
}

int main(void) {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(2000);

    time = to_ms_since_boot(get_absolute_time());
    sleep_ms(1000);

    // configuring Encoder1 A
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) ||             \
    !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a
    // Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

/*
  printf("\nI2C Bus Scan\n");
  printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

  for (int addr = 0; addr < (1 << 7); ++addr) {
    if (addr % 16 == 0) {
      printf("%02x ", addr);
    }

    // Perform a 1-byte dummy read from the probe address. If a slave
    // acknowledges this address, the function returns the number of bytes
    // transferred. If the address byte is ignored, the function returns
    // -1.

    // Skip over any reserved addresses.
    int ret;
    uint8_t rxdata;
    if (reserved_addr(addr))
      ret = PICO_ERROR_GENERIC;
    else
      ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

    printf(ret < 0 ? "." : "@");
    printf(addr % 16 == 15 ? "\n" : "  ");
  }
  printf("Done.\n");
  //return 0;
*/
#endif

    setup_input(MCP_IRQ_GPIO_PIN);
    // configure mcp23008 GPIO as outputs
    write_register_mcp23008(0, 0x00);

    encoder *audio_gain_enc = encoder_init(AUDIO_GAIN_ENC_A, AUDIO_GAIN_ENC_B);
    encoders[0] = audio_gain_enc;

    // audio gain ENC IRQ
    gpio_set_irq_enabled_with_callback(
        AUDIO_GAIN_ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(
        AUDIO_GAIN_ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder *rxgain_enc = encoder_init(RXGAIN_ENC_A, RXGAIN_ENC_B);
    encoders[1] = rxgain_enc;

    // Rx gain ENC IRQ
    gpio_set_irq_enabled_with_callback(
        RXGAIN_ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(
        RXGAIN_ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder *vfo_enc = encoder_init(VFO_ENC_A, VFO_ENC_B);
    encoders[2] = vfo_enc;

    // ENC2 IRQ
    gpio_set_irq_enabled_with_callback(
        VFO_ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(
        VFO_ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder *rit_enc = encoder_init(RIT_ENC_A, RIT_ENC_B);
    encoders[3] = rit_enc;

    // RIT_ENC IRQ
    gpio_set_irq_enabled_with_callback(
        RIT_ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(
        RIT_ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    encoder *filter_enc = encoder_init(FILTER_ENC_A, FILTER_ENC_B);
    encoders[4] = filter_enc;

    // Filter/ZOOM ENC IRQ
    gpio_set_irq_enabled_with_callback(
        FILTER_ENC_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(
        FILTER_ENC_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);


    for (int i = 0; i < 4; i++) {
        gpio_init(cols[i]);
        gpio_set_dir(cols[i], GPIO_IN);
        gpio_pull_down(cols[i]);

        gpio_set_irq_enabled_with_callback(cols[i], GPIO_IRQ_EDGE_RISE, true,
                                           &gpio_callback);
    }

    for (int i = 0; i < 4; i++) {
        gpio_init(rows[i]);
        gpio_set_dir(rows[i], GPIO_OUT);
        gpio_put(rows[i], 1);
    }

    gpio_init(PTT_IN);
    gpio_set_dir(PTT_IN, GPIO_IN);
    ptt_handler();

    MCP23017_GPIOA_val |= POWER_ON_RELAY;
    writemcp23017();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // initialize radio state
    gpio_put(LED_PIN, 1);
    while (true) {
        if (stdio_usb_connected()) {
            break;
        }
        sleep_ms(500);
    }
    gpio_put(LED_PIN, 0);
    radio_state *rs = radio_init();

    struct repeating_timer timer;
    add_repeating_timer_ms(-(500), repeating_timer_callback, NULL, &timer);

    int f = getVFO('A');
    switchLPF(rs, f);

    static unsigned int last_vfo_count;
    while (1) {
        rit_enc_handler(rs, rit_enc);
        rxgain_enc_handler(rs, rxgain_enc);
        audio_gain_enc_handler(rs, audio_gain_enc);
        filter_enc_handler(rs, filter_enc);
        vfo_enc_handler(rs, vfo_enc);

        i2c_expander_handler(rs);

        if (kp_gpio != KPCX)
            keypad_Handler(rs);

        ptt_handler();

        int last_f = f;
        if (timer_ticked) {
            timer_ticked = false;

            // XXX: record VFO encoder counter to measure the pulses
            // turned per unit time. This can be a measure of the
            // velocity of the rotation. Depending on the velocy, the
            // step size can be changed dynamically.
            int vfo_accel = 2 * abs(vfo_enc->count - last_vfo_count); // dividing by 0.5, same as x2
            int step_size_index = 0;
            if (vfo_accel >= 0 && vfo_accel < 100) {
                step_size_index = 1;
            } else if (vfo_accel >= 100 && vfo_accel < 400) {
                step_size_index = 2;
            } else if (vfo_accel >= 400 && vfo_accel < 800) {
                step_size_index = 3;
            } else if (vfo_accel >= 800) {
                step_size_index = 4;
            }
            setStepSize(step_size_index);
            last_vfo_count = vfo_enc->count;

            int f = getVFO('A');
            switchLPF(rs, f);

            // switch between LSB and USB at the 10MHz boundary
            int mode = getMode();
            if (last_f < 10000000 && f >= 10000000 && mode == LSB) {
                // set mode to USB
                printf("ZZMD01;");
            } else if (last_f > 10000000 && f < 10000000 && mode == USB) {
                // set mode to LSB
                printf("ZZMD0;");
            }

            last_f = f;
        }
    }

    return 0;
}
