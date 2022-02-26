# Introduction

This repository contains the code for the Raspberry Pico based
controller for RadioBerry/HL2 designed by Gopan VU2XTO.

For the details of the board etc, please contact Gopan.

# TODO

## PTT-IN

Right now, the PiCo takes a PTT input and sends a command to put
pihpsdr into Tx mode, drives a relay and also turns off the local
audio amplifier. Instead of this signal driving the relay, it would be
good if the PTT out from FPGA drives the relay, since VOX also puts
the radio into Tx mode and this information only lives in SDR
software, not in FPGA, so we can take FPGA PTT out to decide when to
drive PA etc to Tx regardless of the user action on PTT switch or Vox
enable/disable.

# Credits

This code started from a version of the controller done by Dileep
Balan VU2DLE and was then modified by Gopan VU2XTO. The initial
checkin contains both their modifications.
