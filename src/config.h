#ifndef CONFIG_INCLUDE
#define CONFIG_INCLUDE

// define build-time configurable things here

// read PTT out from FPGA
#define PTT_FROM_FPGA_INTO_ADC

// LPF furuno (comment for XTO LPF)
#define LPF_FURUNO

// uncomment the next line if VFO step needs to be decided based on
// the VFO knob turning acceleration.
// #define VFO_ADAPTIVE

// limit step size increments to 1 and 100
#define LIMIT_STEP_INCREMENTS

#define PTT_HANGTIME_MS  150
#endif
