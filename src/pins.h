#ifndef PINS_INCLUDE
#define PINS_INCLUDE

#include "config.h"

#ifdef NEW_BUTTON_MAPPING

#define AUDIO_GAIN_ENC_A 19
#define AUDIO_GAIN_ENC_B 18

#define RIT_ENC_A 12
#define RIT_ENC_B 13

#else

// old mapping (for VU2XTO)

#define AUDIO_GAIN_ENC_A 12 // 19
#define AUDIO_GAIN_ENC_B 13 // 18

#define RIT_ENC_A 19 // 12
#define RIT_ENC_B 18 // 13

#endif

#define RXGAIN_ENC_A 16
#define RXGAIN_ENC_B 17

#define VFO_ENC_A 11
#define VFO_ENC_B 10

#define FILTER_ENC_A 14
#define FILTER_ENC_B 15

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
#define PWM    22

#ifdef NEW_BUTTON_MAPPING

// MCP23017 INPUT
#define ENC_RIT_SW        251 // 0xFB - 1111 1011  (GPB2)
#define ENC_MUTE_DRIVE_SW 254 // 0xFE - 1111 1110  (GPB0)

#else
// old mapping (for VU2XTO)

#define ENC_MUTE_DRIVE_SW 251 // 0xFE - 1111 1110  (GPB0)
#define ENC_RIT_SW        254 // 0xFB - 1111 1011  (GPB2)

#endif

#define ENC_ZOOM_SW       127 // 0x7F - 0111 1111  (GPB7)
#define ENC_RX_RFGAIN_SW  253 // 0xFD - 1111 1101  (GPB1)

#define CW_IAMBIC_DOT     247 // 0xF7 - 1111 0111  (GPB3)
#define CW_IAMBIC_DASH    239 // 0xEF - 1110 1111  (GPB4)

#endif
