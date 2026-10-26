# This repo is for split hoverboard with MM32SPIN0X/2X MCU
 You can help us by testing this code on actual hardware

 Pre-compiled binary is available


## Installation 
Check [here](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/wiki/MM32SPIN05PF%E2%80%90Layout-2.8) for step by step toturial on how to install this firmware on your board

please also check the autodetect&configuration section
## Building from source (GCC)
The firmware builds with the open source Arm GNU toolchain, no Keil or vendor pack needed.
```bash
cd HoverBoardMindMotion
make                                   # build/hoverboard.bin, .hex, .elf
make CROSS=/path/to/bin/arm-none-eabi- # toolchain not on PATH
```
Needs `arm-none-eabi-gcc` (tested with 14.2.Rel1) and `make`. The device headers (MindMotion
register headers, CMSIS) are in `HoverBoardMindMotion/Device/`.

Supported by the GCC build: **MM32SPIN05** (tested on hardware). MM32SPIN06 uses the same
library and may work but is untested. MM32SPIN25 is not ported yet, see
`HoverBoardMindMotion/Device/MM32SPIN2x/Source/README.md`.

## Flashing over SWD
```bash
pip install pyocd
cd HoverBoardMindMotion
make flash                             # = python3 ../tools/mm32flash.py build/hoverboard.bin
```
Works with any probe pyOCD supports (ST-Link, DAPLink, J-Link). The board configuration in the
last 2 KB of flash (written by PinFinder or the RemoteUartBus config command) is kept; the new
firmware is started without a reset so a self-holding board stays powered. A locked (read
protected) chip must be unlocked with a full chip erase first, see the installation guide.

### Watchdog and fault handling
- The independent watchdog runs with a ~0.5 s timeout. A watchdog reset (like any reset) releases
  the power latch, so a hung board switches itself off.
- A fault or unexpected interrupt switches the bridge off (TIM1 MOE) before the watchdog resets.
- Debugging: a breakpoint or halt longer than 0.5 s resets the chip. Set bit 8 (IWDG_STOP) of
  DBGMCU_CR (0x40013404) from the debugger first. On this MM32 the bit freezes the watchdog for as
  long as debug is enabled (until power off), not only while halted. `tools/mm32flash.py` and
  `tools/mm32config.py` handle this themselves. To build without the watchdog, remove
  `#define WATCHDOG` in `Inc/pinout.h`.

## Control methods
The boards are controlled over the RemoteUartBus protocol, see
[PROTOCOL.md](https://gitlab.com/ailife8881/remoteuartbus-online/-/blob/main/PROTOCOL.md). The
[RemoteUartBus web console](https://gitlab.com/ailife8881/remoteuartbus-online) drives and monitors
boards from a browser through a USB-UART adapter.

Several boards can share one bus: master TX to every board's RX, every board's TX to the master RX.
The boards drive TX open drain, so the master RX needs a pull-up (FT232R adapters have one; ESP32:
`pinMode(RX, INPUT_PULLUP)` after `Serial.begin()`). State bit 64 disarms a board (gates off), bits
1–32 set the LEDs.

you can also use a ESP32 to control the hacked hoverboard

you can find example control methods in this repo

https://github.com/reed-dan/hoverboard_hack_esp32_manualspeed
## Working features

The firmware is already 90% functional, basic motor control should all work, but more testing is needed

https://youtu.be/hCaIlEKzI3A
* compatiable with 99.9% of boards without recompile
* working method to unlock and flash the mcu
* motor and hall sensor commutation
* RemoteUartBus protocol(used by gen2.x gd32)
* PID loop for constant speed
* self hold
* voltage and total current
* software and hardware over current protection
* sine wave mode to drive the motor with more torque and higher effiency
## Autodetect&configuration

https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32/-/wikis/home/%7BHow-to-use-PinFinder-firmware%7D

## todo
* optimizations
* constant torque
* phase current
* foc
## Hardware
any board equippted with MM32SPIN0X or MM32SPIN2X microcontroller should be supported

now with the pinfinder firmware, any board type is supported, but below are some example

### [MM32SPIN05PF](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/59)
![301605486-a523f7cc-a34a-4567-8c74-eb735d45fe0a](https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32/assets/142502122/8d725b8a-ae16-4200-9281-23d509e72d12)

### [MM32SPIN06PF](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/61)
![303005959-978f0a8c-0b84-4eb5-9011-176705d9b6e9](https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32/assets/142502122/c4e70bda-5ef6-4b0c-80b7-7830203dcfcc)

### MM32SPIN25PF
![Gen2 6 1](https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32/assets/142502122/7c34e631-0be7-4276-aa93-18d8b7d5c52e)


## support
join this server to get support

discord: https://discord.gg/dvrKS9uyV7

matrix: https://matrix.to/#/#hoverboard-firmware-hack:matrix.ai-life.xyz

## Contributions
if you had any feature request or bug report please create issue or use the [discussions](https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32/discussions) feature, because im working on this project actively now dont use pr unless you asked me first because it will get desynced and cannot merge






