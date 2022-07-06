#ifndef CONFIG_INCLUDE
#define CONFIG_INCLUDE

// define build-time configurable things here

// read PTT out from FPGA
// #define PTT_FROM_FPGA_INTO_ADC

// Ukrainian BPF used by VU2YYF (rest of the folks, comment)
// #define BPF_VU2YYF

// uncomment the next line if VFO step needs to be decided based on
// the VFO knob turning acceleration.
// #define VFO_ADAPTIVE

// limit step size increments to 1 and 1000
#define LIMIT_STEP_INCREMENTS

#define PTT_HANGTIME_MS  200

#endif
