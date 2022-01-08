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

#define PTT_IN 20
#define PWM 22

unsigned long time = 0;
const int delayTime = 50; // Delay for every push button may vary

// MCP23017 INPUT
#define ENC_RIT_SW 251
#define ENC_ZOOM_SW 127
#define ENC_MUTE_DRIVE_SW 254
#define ENC_RX_RFGAIN_SW 253

bool keyer_control = false;
uint32_t int_status;

void waitforradio();

//--------------------------------------------------------------------------

volatile int KeyPressed = false, LongKeyPressed = false, Keyval = 0, old_Keyval;
unsigned int col;
volatile uint kp_gpio = KPCX;
// unsigned int Keyval = 0;
unsigned int keypad_4X4[4][4] = {
    {1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};

// buttons mapping

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

#define PTT_OUT_RELAY 1
#define POWER_ON_RELAY 2
#define BIAS_OUT 32
#define TR_RELAY_OUT 64
#define AM_AMP_MUTE_ON_PTT 128

bool MHZ_enable = false;
uint8_t MCP23017_GPIOA_val = 0;
char zzmd_val[3][3] = {"00\0", "01\0", "06\0"};
char zzmd1_val[3][3] = {"03\0", "04\0", "07\0"};

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
  int rit_increment = 50;

  int mode = getMode();
  if (mode == CWL || mode == CWU) {
      rit_increment = 10;
  } else if (mode == LSB || mode == USB) {
      rit_increment = 100;
  }

  if (ritenc->count != rit_last_count) {
    // Take action here
    if (ritenc->count > rit_last_count) {
      if (rs->rit_val <= -1000)
        rs->rit_val = -1000;
      else {
        // printf("ZZRD;");
        rs->rit_val -= rit_increment;
      }
    } else {
      if (rs->rit_val < 1000) {
        // printf("ZZRU;");
        rs->rit_val += rit_increment;
      } else
        rs->rit_val = 1000;
    }

    rit_last_count = ritenc->count;
    printf("ZZRF%+5d;", rs->rit_val);
    sleep_ms(10);
  }
}

void rxgain_enc_handler(radio_state *rs, encoder *rxgainenc) {
  static unsigned int rxgain_last_count;

  if (!rs->drive_enable) {
    if (rxgainenc->count != rxgain_last_count) {
      // Take action here
      if (rxgainenc->count > rxgain_last_count) {
        rs->rx_gain++;
        if (rs->rx_gain == 48)
          rs->rx_gain = 48;
      } else if (rxgainenc->count < rxgain_last_count) {
        if (rs->rx_gain > -12)
          rs->rx_gain--;
      }
      printf("RA%02d;", rs->rx_gain);
    }
  } else {
    if (rxgainenc->count != rxgain_last_count) {
      // Take action here
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
  }

  rxgain_last_count = rxgainenc->count;
}

void writemcp23017() {
    write_register(MCP23017_GPIOA, ~MCP23017_GPIOA_val);
}

void vfo_enc_handler(radio_state *rs, encoder *vfo_enc) {
  static unsigned int vfo_last_count;

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
          if (rs->audio_gain == 101)
              rs->audio_gain = 100;
      } else if (audio_gain_enc->count > audio_gain_last_count) {
          if (rs->audio_gain > 0)
              rs->audio_gain--;
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
                char vfoB[16];
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
                char vfoB[16];
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

void wait_for_ping(void) {
    char msg[5];
    memset(msg, '\0', 5);

    gpio_put(LED_PIN, 1);
    while (true) {
        for (int i = 0; i < 4; i++) {
            msg[i] = getchar();
        }
        if (strncmp(msg, "PING", 4) == 0) {
            // got the ping message
            break;
        }
    }
    gpio_put(LED_PIN, 0);
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
void I2C_Expander1_Handler(radio_state *rs) {
  // if (interrupt_on_mcp0) {
  static int oldpin = -1;
  interrupt_on_mcp0 = false;
  // int pin = get_last_interrupt_pin();
  // int int_values = get_interrupt_values();
  int input_values_ok; // = update_input_values();
  input_values_ok = read_register(MCP23017_GPIOB);
  if (input_values_ok != oldpin) {
    oldpin = input_values_ok;

    // printf("InputOK:%d\n", input_values_ok);
    switch (input_values_ok) {
    case ENC_RIT_SW: {
      rs->rit = !rs->rit;
      if (rs->rit) {
        printf("ZZRT1;");
      } else {
        printf("ZZRT0;");
      }
      break;
    }
    case ENC_ZOOM_SW: {
      rs->zoom_enable = true;
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
      rs->drive_enable = true;
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
    default:
      rs->zoom_enable = false;
      rs->drive_enable = false;
    }
  }
}
void PTT_Handler() {
  static int old_ptt = -1;
  int ptt = gpio_get(PTT_IN);
  if (old_ptt != ptt) {
    old_ptt = ptt;
    if (ptt == 0) // PTT Pressed
    {
      MCP23017_GPIOA_val &= ~PTT_OUT_RELAY;
      MCP23017_GPIOA_val &= ~TR_RELAY_OUT;
      MCP23017_GPIOA_val |= AM_AMP_MUTE_ON_PTT;
      writemcp23017();
      sleep_ms(20);
      MCP23017_GPIOA_val &= ~BIAS_OUT;
      writemcp23017();
      printf("TX;");

    } else // PTT released
    {
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

// timer tick
/* int globalF = 10000000; //10MHz */
/* bool repeating_timer_callback(struct repeating_timer *t) { */
/*     globalF = getVFO('A'); */

/*     return true; */
/* } */

int main() {
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


    /* ----------------------- Configure 4X4 Keypad
     * ------------------------------------*/
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
    PTT_Handler();
    MCP23017_GPIOA_val |= POWER_ON_RELAY;
    writemcp23017();

    /* ----------------------- Configure 4X4 Keypad
     * ------------------------------------*/

    /* --- while(1) forever scheduler! --- */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // printf("Stating Controller...");

    // before we send any command to the radio, wait for a "custom"
    // handshake between the controller and the radio (in this case
    // the pihpsdr). Once the radio is able to open the serial port,
    // it would send a "PING" message (4 character). Controller would
    // wait for the "PING" string from the serial port before sending
    // any command to the radio.
    // wait_for_ping();

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

    /* struct repeating_timer timer; */
    /* add_repeating_timer_ms(-(1000), repeating_timer_callback, NULL, &timer); */

    int f = getVFO('A');
    switchLPF(rs, f);

    while (1) {
        rit_enc_handler(rs, rit_enc);
        rxgain_enc_handler(rs, rxgain_enc);
        audio_gain_enc_handler(rs, audio_gain_enc);
        filter_enc_handler(rs, filter_enc);
        vfo_enc_handler(rs, vfo_enc);

        I2C_Expander1_Handler(rs);
        // printf("kp_gpio = %d\n", kp_gpio);
        if (kp_gpio != KPCX)
            keypad_Handler(rs);
        PTT_Handler();
    }

    return 0;
}
