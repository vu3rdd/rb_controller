# Introduction

This repository contains the code for the Raspberry Pico based
controller for RadioBerry/HL2 designed by Gopan VU2XTO.

For the details of the board etc, please contact Gopan.

# Building the code

1. [Install pico-sdk](https://github.com/raspberrypi/pico-sdk).
2. clone this repo (`git clone https://github.com/vu3rdd/rb_controller`)
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`

After a successful build, the `.uf2` file path would be
`build/src/rb_controller.uf2`, which can be flashed into the pico.

# Configuration

Since each person may use their own LPF/PA etc, some of the code is
configurable. For instance I (vu2jxn/ex-vu3rdd) use an old Furuno LPF.
The config.h file has some definitions that can be changed at build-time.

# hardware modifications

1. LM386 pin 4 (ground)
2. CN301 pin 3 (ptt out) to pico's pin 32 (ADC1). Pull up resistor
   (3.3k) between pico's pin 32 and pin 36. (I used an opticoupler
   between CN301 and pico)
3. LPF active low vs active high/OC vs non-OC etc..
4. TBD: Iambic keyer

# Pending things to do (2022/Jul/06)

1. Support for user configurable filters (var1/var2).
2. Indication of muted audio.
3. Indication of the slider being changed by the rotary encoder.

# Credits

This code started from a version of the controller done by Dileep
Balan VU2DLE and was then modified by Gopan VU2XTO. The initial
checkin contains both their modifications.
