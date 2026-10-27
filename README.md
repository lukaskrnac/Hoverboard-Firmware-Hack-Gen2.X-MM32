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
Two protocols on the same UART (one baud, config field `baud` in units of 100: 1152 = 115200):

- **CANSimple over UART** (firmware 2.0): ODrive's CANSimple messages and units (turns, turns/s, Nm, A,
  V; float32) in a CRC-protected UART frame, polled per board. It covers set points per control mode,
  encoder/bus/I_q/torque/power feedback, heartbeat with ODrive error bits, Estop/Clear_Errors, and every
  configuration word and runtime value as a parameter (RxSdo/TxSdo, ids in `tools/endpoints.json`).
  Host libraries: `tools/cansimple_uart.py` (Python) and the web console's `protocol.js`.
- **v1 frames** (start `/`): the original Gen2.x RemoteUartBus, unchanged.

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

### Drive modes
`drivemode` (config field 44, or the config frame's iDriveMode) = commutation × 4 + control.
Control 0–3 has the same meaning as the Gen2.x-GD32 firmware's iDrivingMode.

| | pwm (0), permille | speed (1), rpm | torque (2), 0.01 A | position (3), hall steps |
|---|---|---|---|---|
| block (0) | 0 COM_VOLT | 1 COM_SPEED | 2 COM_TORQUE | 3 COM_POSITION |
| sine (1) | 4 SINE_VOLT | 5 SINE_SPEED | 6 SINE_TORQUE | 7 SINE_POSITION |
| FOC (2) | 8 FOC_VOLT | 9 FOC_SPEED | 10 FOC_TORQUE | 11 FOC_POSITION |

- **Command 0:** pwm modes coast (gates off); speed modes brake and hold (block: low-side short
  brake, sine: ramp down then zero vector, FOC: speed loop); torque modes give zero current (FOC)
  or coast (block, sine). Position modes take the command as the target odometer (like the GD32's
  iOdom input) and hold it. When the remote goes silent they hold where they are.
- **Torque:**
  - FOC: measured phase current.
  - Block: a current loop on the bus shunt, which carries the phase current at the sample time in
    block mode.
  - Sine: no current sensing; the voltage is computed from `motorr` and the back EMF.
- **Field weakening** is off by default (CANSimple: `hoverboard.foc.field_weakening_current`,
  `hoverboard.sine.field_weakening_advance`; the console's drive panel has both).
  - FOC: `fwimax`, the most negative d current, at most 90 % of the current limit. A controller sets
    the d current so the current loops keep 5 % voltage headroom. Each amp of d current saves
    omega*L of voltage (~0.37 V at 600 rpm on the bench motor, L ~0.46 mH). Bench wheel at 24 V with
    `current_soft_max` 15 A: 520 rpm without, 626 rpm with 5 A, 745 rpm with 9 A, 836 rpm with 13 A
    (then 12.7 A phase current, 4 A from the supply, unloaded). More speed needs more d current,
    i.e. a higher current limit.
  - Sine: `fwadvance`, phase advance in degrees. Sine field weakening has no current control and is
    expensive; see `Inc/pinout.h`.
  - Above the speed the battery voltage allows, the back EMF is higher than the bus: a coast (gates
    off) then charges the bus through the MOSFET diodes, which a bench supply cannot take. Ramp down
    (VEL_RAMP; voltage commands slew at 1 ‰/ms) instead of stopping at once.
- **Overvoltage:** the board coasts while the bus is 3 V above `batfull`, so braking doesn't pump a
  bench supply.

Configs saved before the version field (word 63) used 2/3 for SINE_VOLT/SINE_SPEED; they are
migrated at boot. FOC needs the two phase current pins (`iphasea`/`iphaseb`, `iphasecfg` selects the
phases and polarity).

### Phase currents on the MM32SPIN05PFOP
The PFOP variant has two internal op-amps that MindMotion does not document for the SPIN05. They work
without any register setup. On the bench board they amplify the low-side current of phases B and C:
outputs PA2/PA6, inputs PA0/PA1 and PA4/PA5, about 19.4 ADC counts per A and an offset of ~1520
counts. The outputs are only valid while the low sides conduct, and they saturate when the gates are
off. Config for that board: `iphasea=PA2 iphaseb=PA6 iphasecfg=1`.

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
* FOC with two measured phase currents; pwm, speed, torque and position control for block, sine and FOC; field weakening
## Autodetect&configuration

https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32/-/wikis/home/%7BHow-to-use-PinFinder-firmware%7D

## todo
* optimizations
* bus current calibration
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






