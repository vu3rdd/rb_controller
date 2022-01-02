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

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
#endif

#define ENC1A 19
#define ENC1B 18

#define ENC2A 16
#define ENC2B 17

#define ENC3A 11
#define ENC3B 10

#define ENC4A 12
#define ENC4B 13

#define ENC5A 14
#define ENC5B 15

#define KPR0 2
#define KPR1 3
#define KPR2 5
#define KPR3 7

#define KPC0 0
#define KPC1 1
#define KPC2 4
#define KPC3 6
#define KPCX 99 // undefined column number!

#define PTT_IN 20
#define PWM 22

unsigned long time = 0;
const int delayTime = 50; // Delay for every push button may vary

// MCP23017 INPUT
#define ENC_RIT_SW 251
#define ENC_ZOOM_SW 127
#define ENC_MUTE_DRIVE_SW 254
#define ENC_RX_RFGAIN_SW 253

#define FSTEP 191
#define RXANT 223

bool keyer_control = false;
bool mute = false;

uint32_t int_status;

void waitforradio();

//--------------------------------------------------------------------------
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#define R_START 0x0
//#define HALF_STEP

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
    // R_START (00)
    {R_START_M, R_CW_BEGIN, R_CCW_BEGIN, R_START},
    // R_CCW_BEGIN
    {R_START_M | DIR_CCW, R_START, R_CCW_BEGIN, R_START},
    // R_CW_BEGIN
    {R_START_M | DIR_CW, R_CW_BEGIN, R_START, R_START},
    // R_START_M (11)
    {R_START_M, R_CCW_BEGIN_M, R_CW_BEGIN_M, R_START},
    // R_CW_BEGIN_M
    {R_START_M, R_START_M, R_CW_BEGIN_M, R_START | DIR_CW},
    // R_CCW_BEGIN_M
    {R_START_M, R_CCW_BEGIN_M, R_START_M, R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const unsigned char ttable[7][4] = {
    // R_START
    {R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START},
    // R_CW_FINAL
    {R_CW_NEXT, R_START, R_CW_FINAL, R_START | DIR_CW},
    // R_CW_BEGIN
    {R_CW_NEXT, R_CW_BEGIN, R_START, R_START},
    // R_CW_NEXT
    {R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START},
    // R_CCW_BEGIN
    {R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START},
    // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | DIR_CCW},
    // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

unsigned char ENC1state, ENC2state, ENC3state, ENC4state, ENC5state;
volatile int ENC1NewState, ENC2NewState, ENC3NewState, ENC4NewState,
    ENC5NewState;
volatile int KeyPressed = false, LongKeyPressed = false, Keyval = 0, old_Keyval;
unsigned int col;
volatile uint kp_gpio = KPCX;
// unsigned int Keyval = 0;
unsigned int keypad_Row[4] = {KPR0, KPR1, KPR2, KPR3};
unsigned int keypad_4X4[4][4] = {
    {1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};

#define ON_OFF 13
#define ANT_SEL 9
#define VOX 5         //

#define BAND_DWN 1    //
#define BAND_UP 2     //
#define LSB_USB_AM 3  //
#define LCW_UCW_DIG 4 //

#define VFO_A_B 7     //
#define MHZ_STEP 10   //
#define VFO_SWAP 6    //
#define SPLIT 8       //
#define NB 11         //
#define NR 12         //
#define AGC 14      //
#define CTUNE 15      //
#define FLOCK 16      //

#define PTT_OUT_RELAY 1
#define POWER_ON_RELAY 2
#define BIAS_OUT 32
#define TR_RELAY_OUT 64
#define AM_AMP_MUTE_ON_PTT 128

int ctune = 0;
int zzmd_index = 0;
int rx_gain = 0;
int tx_gain = 50;
int audio_gain = 25;
int zzac_index = 0;
int nb_val = 0;
int snb_val = 0;
int nr_val = 0;
int vox_val = 0;
int split_val = 0;
int lock_val = 0;
int filter_val = 0;
int rit_val = 0;
bool rit = false;
bool zoom_enable = false;
bool drive_enable = false;
int agc_mode = 0;
int zoom_val = 0;
long long f = 0;
uint8_t antsel = 0;
uint8_t rxant = 0;
uint8_t lpf = 0;
uint8_t MCP23017_GPIOA_val = 0;
char zzmd_val[3][3] = {"00\0", "01\0", "06\0"};
int zzmd1_index = 0;
char zzmd1_val[3][3] = {"03\0", "04\0", "07\0"};
bool power = false;
bool MHZ_enable = false;

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

void RIT_ENC_Handler() { // RIT
  int s = 0;
  int_status = save_and_disable_interrupts();
  if (ENC4NewState == 32) {
    s = 1;
  }
  if (ENC4NewState == 16) {
    s = -1;
  }
  ENC4NewState = 0;
  restore_interrupts(int_status);

  if (s != 0) {
    // Take action here
    if (s == -1) {

      if (rit_val <= -1000)
        rit_val = -1000;
      else {
        // printf("ZZRD;");
        rit_val -= 100;
      }

    } else if (s == 1) {

      if (rit_val < 1000) {
        // printf("ZZRU;");
        rit_val += 100;
      } else
        rit_val = 1000;
    }

    printf("ZZRF%+5d;", rit_val);
    sleep_ms(10);
  }

}
void ENC2_Handler() { // RX Gain
  int s = 0;
  int_status = save_and_disable_interrupts();
  if (ENC2NewState == 32) {
    s = 1;
  }
  if (ENC2NewState == 16) {
    s = -1;
  }
  ENC2NewState = 0;
  restore_interrupts(int_status);
  if (!drive_enable) {
    if (s != 0) {
      // Take action here
      if (s == -1) {
        rx_gain++;
        if (rx_gain == 48)
          rx_gain = 48;
      } else if (s == 1) {
        if (rx_gain > -12)
          rx_gain--;
      }
      printf("RA%02d;", rx_gain);
    }
  } else {
    if (s != 0) {
      // Take action here
      if (s == 1) {
        tx_gain++;
        if (tx_gain >= 100)
          tx_gain = 100;
      } else if (s == -1) {
        if (tx_gain > 0)
          tx_gain--;
      }
      printf("PC%03d;", tx_gain);
    }
  }
}
void writemcp23017() {
    write_register(MCP23017_GPIOA, ~MCP23017_GPIOA_val);
}

void writetomcp23008() {
    write_register_mcp23008(9, lpf | antsel | rxant);
}

bool readFrequency() {
  // return;
  char freqbuff[20];
  memset(freqbuff, '0', 16);
  int cnt = 0;
  static uint8_t oldlpf = 0;
  printf("ZZFA;");
  // sleep_ms(15);
  for (cnt = 0; cnt < 16; cnt++) {
    // sleep_ms(1);
    freqbuff[cnt] = getchar();
    if (freqbuff[cnt] == ';')
      break;
  }
  if (freqbuff[15] == ';') {
    f = atoll(&freqbuff[4]);
    if (f < 2e6) {
      lpf = 1;
    } else if (f < 3e6) {
      lpf = 2;
    } else if (f < 5e6) {
      lpf = 4;
    } else if (f < 9e6) {
      lpf = 8;
    } else if (f < 16e6) {
      lpf = 16;
    } else if (f <= 30e6) {
      lpf = 32;
    }
    if (lpf != oldlpf) {
      writetomcp23008();
      oldlpf = lpf;
    }
    return true;
  } else
    return false;
}

void ENC3_Handler() { // VFO Up-Down
  int s = 0;
  int_status = save_and_disable_interrupts();
  if (ENC3NewState == 32) {
    s = 1;
  }
  if (ENC3NewState == 16) {
    s = -1;
  }
  ENC3NewState = 0;
  restore_interrupts(int_status);

  if (s != 0) {
    // Take action here
    if (s == 1) {
      printf("ZZAF01;");
    }
    if (s == -1) {
      printf("ZZAE01;");
    }
    // sleep_ms(10);
    readFrequency();
  }
}
void Audio_ENC_Handler() { // Audio
  int s = 0;
  int_status = save_and_disable_interrupts();
  if (ENC1NewState == 32) {
    s = 1;
  }
  if (ENC1NewState == 16) {
    s = -1;
  }
  ENC1NewState = 0;
  restore_interrupts(int_status);

  if (s != 0) {
    // Take action here
    if (s == 1) {
      audio_gain++;
      if (audio_gain == 101)
        audio_gain = 100;
    } else if (s == -1) {
      if (audio_gain > 0)
        audio_gain--;
    }
    printf("ZZAG%03d;", audio_gain);
  }
}

void ENC5_Handler() { // Zoom - Filter
  int s = 0;
  int_status = save_and_disable_interrupts();
  if (ENC5NewState == 32) {
    s = 1;
  }
  if (ENC5NewState == 16) {
    s = -1;
  }
  ENC5NewState = 0;
  restore_interrupts(int_status);
  if (s != 0) {
    if (!zoom_enable) {
      // Take action here
      if (s == 1) {
        filter_val += 1;
        if (filter_val >= 8)
          filter_val = 8;
      } else {
        if (filter_val > 0)
          filter_val--;
        else
          filter_val = 0;
      }
      printf("ZZFI%02d;", filter_val);
      // printf("FW%04d;", filter_val);
    } else {
      if (s == -1) {
        zoom_val++;
        if (zoom_val > 8)
          zoom_val = 8;
      } else {
        if (zoom_val <= 1)
          zoom_val = 1;
        else
          zoom_val--;
      }
      printf("ZZPY%03d;", zoom_val);
    }
  }
}

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

void keypad_Handler() {
    gpio_put(KPR0, 0);
    gpio_put(KPR1, 0);
    gpio_put(KPR2, 0);
    gpio_put(KPR3, 0);

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
        gpio_put(keypad_Row[row], 1);
        sleep_ms(10);
        if (gpio_get(kp_gpio)) {
            Keyval = keypad_4X4[row][col];
            KeyPressed = true;
            sleep_ms(250);
            if (gpio_get(kp_gpio)) {
                LongKeyPressed = true;
            }
        }
        gpio_put(keypad_Row[row], 0);
    }

    gpio_put(KPR0, 1);
    gpio_put(KPR1, 1);
    gpio_put(KPR2, 1);
    gpio_put(KPR3, 1);

    if (KeyPressed) {
        /* printf("kv = %d\n",Keyval); */
        /* Keyval = 0; KeyPressed = false; */
        /* kp_gpio = KPCX; */

        /* return; */
        // keypad_msg[Keyval - 1][4] = '\0';
        switch (Keyval) {
        case CTUNE:
            printf("ZZCN%d;", ctune);
            if (ctune == 0)
                ctune = 1;
            else
                ctune = 0;
            break;
        case LSB_USB_AM:
            printf("ZZMD%s;", zzmd_val[zzmd_index]);
            zzmd_index++;
            if (zzmd_index == 3)
                zzmd_index = 0;
            break;
        case LCW_UCW_DIG:
            printf("ZZMD%s;", zzmd1_val[zzmd1_index]);
            zzmd1_index++;
            if (zzmd1_index == 3)
                zzmd1_index = 0;
            break;
        case AGC: {
            switch (agc_mode) {
            case 0:
                agc_mode = 2;
                break;
            case 2:
                agc_mode = 3;
                break;
            case 3:
                agc_mode = 4;
                break;
            case 4:
                agc_mode = 2;
                break;
            default:
                agc_mode = 2;
            }
            printf("ZZGT%d;", agc_mode);
            break;
        }
        case BAND_UP:
            if (!MHZ_enable) {
                printf("ZZBU;");
                sleep_ms(15);
                readFrequency();
            } else {
                f += 1e6;
                if (f > 30e6)
                    f = 30e6;
                printf("ZZFA%011lld;", f);
                // sleep_ms(15);
                readFrequency();
            }
            break;
        case BAND_DWN:
            if (!MHZ_enable) {
                printf("ZZBD;");
                // sleep_ms(15);
                readFrequency();
            } else {
                if (f > 1e6)
                    f -= 1e6;
                printf("ZZFA%011lld;", f);
                sleep_ms(15);
                readFrequency();
            }
            break;
        case NB:
            if (LongKeyPressed) {
                snb_val = (snb_val + 1) % 2;
                if (snb_val == 1) {
                    printf("ZZNN1;");
                } else {
                    printf("ZZNN0;");
                }
                break;
            }
            // cycle nb value between 0, 1 and 2.
            nb_val = (nb_val + 1) % 3;
            switch (nb_val) {
            case 0:
                printf("ZZNA0;ZZNB0;");
                break;
            case 1:
                printf("ZZNA1;ZZNB0;");
                break;
            case 2:
                printf("ZZNA0;ZZNB1;");
                break;
            }
            break;
        case NR:
            nr_val = (nr_val + 1) % 3;
            switch (nr_val) {
            case 0:
                printf("ZZNR0;ZZNS0;");
                break;
            case 1:
                printf("ZZNR1;ZZNS0;");
                break;
            case 2:
                printf("ZZNR0;ZZNS1;");
                break;
            }
            break;
        case VFO_A_B:
            printf("ZZVS0;");
            break;
        case MHZ_STEP:
            if (MHZ_enable) {
                MHZ_enable = false;
                // printf("ZZTI0;");
            } else {
                MHZ_enable = true;
                // printf("ZZTI1;");
            }
            // printf("ZZVS1;");
            break;
        case VFO_SWAP:
            printf("ZZVS2;");
            readFrequency();
            break;
        case VOX:
            if (vox_val == 0)
                vox_val = 1;
            else
                vox_val = 0;
            printf("VX%d;", vox_val);
            break;
        case SPLIT:
            // first read the current mode. If the mode is CW, then VFOB=VFOA+1kc and SPLIT ON
            // if mode is SSB (LSB/USB), then VFOB = vfoA + 5kc and SPLIT ON
            printf("ZZMD;"); // try to read the current mode
            char current_mode[20];
            memset(current_mode, '\0', 20);
            for (int cnt = 0; cnt < 20; cnt++) {
                current_mode[cnt] = getchar();
                if (current_mode[cnt] == ';')
                    break;
            }

            // for debug
            // printf("response: %s", current_mode);

            // check if mode is CW
            if (strncmp(current_mode, "ZZMD03;", 7) == 0 ||
                strncmp(current_mode, "ZZMD04;", 7) == 0) {
                // vfoB = vfoA + 1khz
                // first read vfo A
                int vfoA = getVFO('A');

                // calculate the new vfo B value
                int vfoB_int = vfoA + 1000;

                // set vfo B value
                char vfoB[16];
                sprintf(vfoB, "ZZFB%011d;", vfoB_int);
                printf("%s", vfoB);
            } else if (strncmp(current_mode, "ZZMD0;", 6) == 0 ||
                       strncmp(current_mode, "ZZMD00;", 7) == 0 ||
                       strncmp(current_mode, "ZZMD01;", 7) == 0) {
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

            if (split_val == 0)
                split_val = 1;
            else
                split_val = 0;
            printf("ZZSP%d;", split_val);
            break;
        case FLOCK:
            if (lock_val == 0)
                lock_val = 1;
            else
                lock_val = 0;
            printf("ZZVL%d;", lock_val);
            break;
        case ANT_SEL:
            if (antsel == 0)
                antsel = 64;
            else
                antsel = 0;
            write_register(MCP23017_GPIOA, ((antsel == 1) ? 4 : 8));
            writetomcp23008();
            break;
        case ON_OFF:
            if (power) {
                printf("#S;");
                sleep_ms(10000);
                MCP23017_GPIOA_val |= POWER_ON_RELAY;
                writemcp23017();
                sleep_ms(20);
                power = false;
            } else {
                MCP23017_GPIOA_val &= ~POWER_ON_RELAY;
                writemcp23017();
                sleep_ms(20);
                waitforradio();
                power = true;
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

void waitforradio() {
  // return;
  if (1 == 1) {
    for (int x = 0; x < 30; x++) {
      gpio_put(LED_PIN, 1);
      sleep_ms(500);
      gpio_put(LED_PIN, 0);
      sleep_ms(500);
    }
    readFrequency();
  }
}

//--------------------------------------------------------------------------
void ENC1_callback_Handler(void) {
  unsigned char pinstate = (gpio_get(ENC1B) << 1) | gpio_get(ENC1A);
  // Determine new state from the pins and state table.
  ENC1state = ttable[ENC1state & 0xf][pinstate];
  // Return emit bits, ie the generated event
  pinstate = ENC1state & 0x30;
  if (pinstate) {
    ENC1NewState = (int)pinstate;
  }
}
void ENC2_callback_Handler(void) {
  unsigned char pinstate = (gpio_get(ENC2B) << 1) | gpio_get(ENC2A);
  // Determine new state from the pins and state table.
  ENC2state = ttable[ENC2state & 0xf][pinstate];
  // Return emit bits, ie the generated event
  pinstate = ENC2state & 0x30;
  // printf("pinstate = %d\n", pinstate);
  if (pinstate) {
    ENC2NewState = (int)pinstate;
  }
}
void ENC3_callback_Handler(void) {
  unsigned char pinstate = (gpio_get(ENC3B) << 1) | gpio_get(ENC3A);
  // Determine new state from the pins and state table.
  ENC3state = ttable[ENC3state & 0xf][pinstate];
  // Return emit bits, ie the generated event
  pinstate = ENC3state & 0x30;
  // printf("pinstate = %d\n", pinstate);
  if (pinstate) {
    ENC3NewState = (int)pinstate;
  }
}

void ENC4_callback_Handler(void) {
  unsigned char pinstate = (gpio_get(ENC4B) << 1) | gpio_get(ENC4A);
  // Determine new state from the pins and state table.
  ENC4state = ttable[ENC4state & 0xf][pinstate];
  // Return emit bits, ie the generated event
  pinstate = ENC4state & 0x30;
  // printf("pinstate = %d\n", pinstate);
  if (pinstate) {
    ENC4NewState = (int)pinstate;
  }
}
void ENC5_callback_Handler(void) {
  unsigned char pinstate = (gpio_get(ENC5B) << 1) | gpio_get(ENC5A);
  // Determine new state from the pins and state table.
  ENC5state = ttable[ENC5state & 0xf][pinstate];
  // Return emit bits, ie the generated event
  pinstate = ENC5state & 0x30;
  // printf("pinstate = %d\n", pinstate);
  if (pinstate) {
    ENC5NewState = (int)pinstate;
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

    if ((gpio == ENC1A) || (gpio == ENC1B)) {
        ENC1_callback_Handler();
    } else if ((gpio == ENC2A) || (gpio == ENC2B)) {
        ENC2_callback_Handler();
    } else if ((gpio == ENC3A) || (gpio == ENC3B)) {
        ENC3_callback_Handler();
    } else if ((gpio == ENC4A) || (gpio == ENC4B)) {
        ENC4_callback_Handler();
    } else if ((gpio == ENC5A) || (gpio == ENC5B)) {
        ENC5_callback_Handler();
    }
    restore_interrupts(int_status);
}
void I2C_Expander1_Handler() {
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
      rit = !rit;
      if (rit) {
        printf("ZZRT1;");
      } else {
        printf("ZZRT0;");
      }
      break;
    }
    case ENC_ZOOM_SW: {
      zoom_enable = true;
      break;
    }
    case ENC_MUTE_DRIVE_SW: {
      mute = !mute;
      if (mute) {

        printf("ZZMA1;");
      } else {
        printf("ZZMA0;");
      }

      break;
    }
    case ENC_RX_RFGAIN_SW: {
      drive_enable = true;
      break;
    }

    case FSTEP: {
        // cycle through 1, 10, 100, 1000 hz
        switch (zzac_index) {
        case 0: // 1 hz -> 10 hz
            zzac_index = 1;
            break;
        case 1: // 10 hz -> 100 hz
            zzac_index = 4;
            break;
        case 4: // 100 hz -> 1khz
            zzac_index = 7;
            break;
        case 7: // 1khz -> 10k hz
            zzac_index = 13;
            break;
        case 13: // 10khz -> 100k hz
            zzac_index = 20;
            break;
        case 20: // 100khz -> 1Mhz
            zzac_index = 23;
            break;
        case 23: // 1Mhz -> 1hz
            zzac_index = 0;
            break;
        default:
            zzac_index = 0;
            break;
        }
        printf("ZZAC%02d;", zzac_index);
        break;
    }
    case RXANT: {
        if (rxant == 0)
            rxant = 128;
        else
            rxant = 0;
        writetomcp23008();
        break;
    }
     /*  switch (agc_mode) { */
    /*   case 0: */
    /*     agc_mode = 2; */
    /*     break; */
    /*   case 2: */
    /*     agc_mode = 3; */
    /*     break; */
    /*   case 3: */
    /*     agc_mode = 4; */
    /*     break; */
    /*   case 4: */
    /*     agc_mode = 2; */
    /*     break; */
    /*   default: */
    /*     agc_mode = 2; */
    /*   } */
    /*   printf("ZZGT%d;", agc_mode); */
    /*   break; */

    default:
      zoom_enable = false;
      drive_enable = false;
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

int main() {

  stdio_init_all();

  sleep_ms(2000);
  // printf("Initislizing Controller...");
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
  write_register_mcp23008(0, 0x00);

  gpio_init(ENC1A);
  gpio_set_dir(ENC1A, GPIO_IN);
  gpio_pull_up(ENC1A);
  // configuring Encoder1 B
  gpio_init(ENC1B);
  gpio_set_dir(ENC1B, GPIO_IN);
  gpio_pull_up(ENC1B);
  // ENC1 IRQ
  gpio_set_irq_enabled_with_callback(
      ENC1A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(
      ENC1B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // configuring Encoder2 A
  gpio_init(ENC2A);
  gpio_set_dir(ENC2A, GPIO_IN);
  gpio_pull_up(ENC2A);
  // configuring Encoder2 B
  gpio_init(ENC2B);
  gpio_set_dir(ENC2B, GPIO_IN);
  gpio_pull_up(ENC2B);
  // ENC2 IRQ
  gpio_set_irq_enabled_with_callback(
      ENC2A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(
      ENC2B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // configuring Encoder3 A
  gpio_init(ENC3A);
  gpio_set_dir(ENC3A, GPIO_IN);
  gpio_pull_up(ENC3A);
  // configuring Encoder3 B
  gpio_init(ENC3B);
  gpio_set_dir(ENC3B, GPIO_IN);
  gpio_pull_up(ENC3B);
  // ENC3 IRQ
  gpio_set_irq_enabled_with_callback(
      ENC3A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(
      ENC3B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // configuring Encoder4 A
  gpio_init(ENC4A);
  gpio_set_dir(ENC4A, GPIO_IN);
  gpio_pull_up(ENC4A);
  // configuring Encoder4 B
  gpio_init(ENC4B);
  gpio_set_dir(ENC4B, GPIO_IN);
  gpio_pull_up(ENC4B);
  // ENC4 IRQ
  gpio_set_irq_enabled_with_callback(
      ENC4A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(
      ENC4B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  // configuring Encoder5 A
  gpio_init(ENC5A);
  gpio_set_dir(ENC5A, GPIO_IN);
  gpio_pull_up(ENC5A);
  // configuring Encoder5 B
  gpio_init(ENC5B);
  gpio_set_dir(ENC5B, GPIO_IN);
  gpio_pull_up(ENC5B);
  // ENC5 IRQ
  gpio_set_irq_enabled_with_callback(
      ENC5A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(
      ENC5B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  /* ----------------------- Configure 4X4 Keypad
   * ------------------------------------*/
  gpio_init(KPC0);
  gpio_set_dir(KPC0, GPIO_IN);
  gpio_pull_down(KPC0);
  gpio_init(KPC1);
  gpio_set_dir(KPC1, GPIO_IN);
  gpio_pull_down(KPC1);
  gpio_init(KPC2);
  gpio_set_dir(KPC2, GPIO_IN);
  gpio_pull_down(KPC2);
  gpio_init(KPC3);
  gpio_set_dir(KPC3, GPIO_IN);
  gpio_pull_down(KPC3);
  gpio_set_irq_enabled_with_callback(KPC0, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_callback);
  gpio_set_irq_enabled_with_callback(KPC1, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_callback);
  gpio_set_irq_enabled_with_callback(KPC2, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_callback);
  gpio_set_irq_enabled_with_callback(KPC3, GPIO_IRQ_EDGE_RISE, true,
                                     &gpio_callback);

  gpio_init(KPR0);
  gpio_set_dir(KPR0, GPIO_OUT);
  gpio_put(KPR0, 1);
  gpio_init(KPR1);
  gpio_set_dir(KPR1, GPIO_OUT);
  gpio_put(KPR1, 1);
  gpio_init(KPR2);
  gpio_set_dir(KPR2, GPIO_OUT);
  gpio_put(KPR2, 1);
  gpio_init(KPR3);
  gpio_set_dir(KPR3, GPIO_OUT);
  gpio_put(KPR3, 1);

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

  while (1) {
      RIT_ENC_Handler();
      ENC2_Handler();
      ENC3_Handler();
      Audio_ENC_Handler();
      ENC5_Handler();
      I2C_Expander1_Handler();
      // printf("kp_gpio = %d\n", kp_gpio);
      if (kp_gpio != KPCX)
          keypad_Handler();
      PTT_Handler();
  }
  return 0;
}
