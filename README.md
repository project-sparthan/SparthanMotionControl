# SparthanMotionControl
Firmware for the STM32 motion controller found on the SPARTHAN control board v2

## Toolchain Installation on Windows
Toolchain on Windows

### miniGW MAKE
- download minGW installer
- install all basic packages except for ada, fortran, objc
- add 'C:\MinGW\msys\1.0\bin' to your system path variable
- launch cmd, type in 'make'
- if the cmd terminal returns 'make: *** No targets specified and no makefile found.  Stop.', then you have done it right.

### Optional if you want to compile C programs
- add 'C:\MinGW\bin' to your system path variable
- launch cmd, type in 'gcc'
- if the cmd terminal returns 'gcc: fatal error: no input files compilation terminated.', then you have done it right

### ARM GCC toolchain
- download gcc-arm-none-eabi
- install with installer
- check the 'Add path to environment variable' option
- launch cmd, type in 'arm-none-eabi-gcc'
- if the cmd terminal returns 'arm-none-eabi-gcc: fatal error: no input files compilation terminated.', then you have done it right

### Testing everything
- get a makefile based stm32 project
- launch a cmd terminal in the folder where the makefile is located
- type 'make', it should build
