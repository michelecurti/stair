# stair

Switch on a stair that for every step has a strip led, 
controlled in PWM.
All this to generate a stair "wave effect" for my sister.

## project structure

under hw there is the hardware
under fw there is the firmware

Meh..

## software requirements

I worked on OpenBSD.

For the hardware I used Kicad, for the firmware, vim + Makefile..

The toolchain is arm-none-eabi-gcc-linaro.

The "core" is the STM32 BluePill, some shift registers and some
mosfets..

To debug the board I used the st-link with openocd.

## how to compile

$ cd fw/stair/
$ make

## how to debug

$ make debug
