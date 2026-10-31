# Hoverboard Firmware Hack Gen2.x MM32 — open-source FOC motor controller firmware for MindMotion MM32SPIN05 / SPIN06 / SPIN25 / MM32F031 hoverboard boards

Open-source replacement firmware for **split (Gen2.x) hoverboard mainboards** that use a **MindMotion MM32**
microcontroller (MM32SPIN05PF, MM32SPIN06PF, MM32SPIN25PF, MM32F031). It turns a hoverboard half into a
**BLDC / hub motor controller for robots, rovers, carts and DIY electric vehicles**, controlled over UART.

- **Control:** block (trapezoidal), sine and **field oriented control (FOC)**, each with pwm, speed, torque
  and position modes; field weakening; regenerative braking with bus voltage limiting.
- **Protocols:** **ODrive CANSimple over UART** (ODrive names and units: turns, turns/s, Nm, A) and the
  original Gen2.x **RemoteUartBus** v1 protocol; several boards on one bus.
- **Any board, no recompile:** the **PinFinder** firmware auto-detects hall sensors, phase / bus current pins,
  battery divider, buttons, power latch, LEDs, buzzer, charger, foot sensors and an I2C IMU, and stores the
  configuration in flash.
- **Toolchain:** plain GCC (`arm-none-eabi-gcc`) + `make`, flashing with **pyOCD** over SWD (ST-Link,
  DAPLink, J-Link). No Keil, no vendor tools.
- **License:** MIT.

Demo video: https://youtu.be/hCaIlEKzI3A

Related projects: [Hoverboard-Firmware-Hack-Gen2.x (GD32)](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x),
[RemoteUartBus web console](https://gitlab.com/ailife8881/remoteuartbus-online) (browser control, PinFinder
setup wizard, per-board image builder).

The previous Keil-based firmware is kept in the [`legacy-keil`](https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32/-/tree/legacy-keil)
branch (tag `legacy-keil-final`); it is no longer maintained.

---

## Contents
1. [Supported hardware](#supported-hardware)
2. [Quick start](#quick-start)
3. [Building](#building)
4. [Flashing over SWD](#flashing-over-swd)
5. [PinFinder: auto-detect and configure a board](#pinfinder-auto-detect-and-configure-a-board)
6. [Controlling the motor](#controlling-the-motor)
7. [Drive modes](#drive-modes)
8. [Settings](#settings)
9. [Peripherals: IMU, buzzer, charger, foot sensors](#peripherals-imu-buzzer-charger-foot-sensors)
10. [Safety: watchdog, faults, current and voltage limits](#safety-watchdog-faults-current-and-voltage-limits)
11. [Repository layout](#repository-layout)
12. [FAQ](#faq)
13. [Support and contributing](#support-and-contributing)

---

## Supported hardware

Any Gen2.x split hoverboard board with one of these MCUs. The pins are configured at runtime, so one image
per MCU covers every board layout.

| MCU | `make MCU=` | Status | Notes |
|---|---|---|---|
| **MM32SPIN05PF / PFOP** | `SPIN05` (default) | tested on hardware | PFOP: two internal phase current op-amps (PA2/PA6), no setup needed |
| **MM32F031** | `F031` | tested on hardware | same die as the SPIN05 (IDCODE 0xCC4460B1) without the gate driver |
| MM32SPIN06PF | `SPIN06` | builds, untested | SPIN05 pin tables |
| MM32SPIN25PF | `SPIN25` | builds, untested | 72 MHz PLL; op-amps OP1-3 enabled per configured current pin |

All four share the SPIN05 register layout (compared in the vendor SVD files). Every image uses a 32 KB flash
layout (31 KB code, the configuration in the last 1 KB page) and 4 KB of RAM. The PWM pins must be
PA8-PA10 / PB13-PB15 (TIM1, AF2).

Board photos: [MM32SPIN05PF](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/59),
[MM32SPIN06PF](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/61). SWD pads:
[extraFiles/swd-mm32.png](extraFiles/swd-mm32.png). Unlocking a read-protected chip:
[installation guide](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/wiki/MM32SPIN05PF%E2%80%90Layout-2.8).

## Quick start

```bash
# 1. tools: Arm GNU toolchain (arm-none-eabi-gcc, tested 14.2.Rel1), make, python3, pyOCD
pip install pyocd

# 2. build PinFinder + main firmware for your MCU (from the repo root)
make MCU=SPIN05 both

# 3. flash PinFinder, detect the pins over the UART, save
make MCU=SPIN05 flash-pinfinder
python3 tools/pinfinder.py info "detect hall" "detect vbat 36500" "detect current" save

# 4. flash the main firmware (the configuration page is kept)
make MCU=SPIN05 flash
```

## Building

The top-level `Makefile` forwards to `HoverBoardMindMotion/Makefile`; both work from either directory.

| Command | Result |
|---|---|
| `make` | main firmware for the MM32SPIN05: `HoverBoardMindMotion/build/spin05/hoverboard.{elf,bin,hex}` |
| `make MCU=F031` | another MCU (`SPIN05`, `SPIN06`, `SPIN25`, `F031`) → `build/<mcu>/` |
| `make pinfinder` | PinFinder → `build/<mcu>/pinfinder.{elf,bin,hex}` |
| `make both` | main firmware and PinFinder for `MCU` |
| `make targets` | main firmware and PinFinder for every MCU |
| `make release` | `targets`, then all images collected in `release/<mcu>/` |
| `make CROSS=/path/to/bin/arm-none-eabi-` | toolchain not on `PATH` |
| `make TRACE=1` | with the FOC capture buffer for SWD tools (768 B RAM) |
| `make clean` / `make clean-all` | this MCU's build / every build and `release/` |
| `make help` | this list |

A change of flags (`TRACE`, `CROSS`) rebuilds everything. Integer division uses the chip's hardware divider
(`Src/hwdiv.S`), floats a compact soft float (`Src/softfloat.c`); `-flto -Os`. The flash is nearly full
(~31.6 of 31 KB used on every MCU), so new features need equal savings.

Precompiled images are not kept in git; build them with `make release` or take them from the project's
releases.

## Flashing over SWD

```bash
make MCU=SPIN05 flash             # = python3 tools/mm32flash.py HoverBoardMindMotion/build/spin05/hoverboard.bin
make MCU=SPIN05 flash-pinfinder
python3 tools/mm32flash.py image.bin --backup before.bin   # dump all 32 KB first
python3 tools/mm32flash.py image.bin --erase-config        # also wipe the board configuration
```

Works with any probe pyOCD supports. pyOCD has no MM32 flash algorithm: `mm32flash.py` erases through the
FLASH registers and programs with a small RAM routine. The board configuration (last 1 KB page) is kept, and
the new firmware starts without a reset so a self-holding board stays powered. A read-protected chip needs a
full chip erase first (installation guide above).

## PinFinder: auto-detect and configure a board

PinFinder is a separate firmware (the main firmware has no room for it) that finds the board's pins and
writes the configuration the main firmware boots from. It talks 115200 8N1. On a board without a
configuration it first searches the UART pins: short the board's TX and RX (loop-back) until it finds them,
then connect your USB-UART adapter. It prints `PinFinder 2.0: Enter = menu, @<command> = agent mode`.

- **TUI** (press Enter): pin map, voltages, menu for each detection, test motor, save.
- **Agent mode** (lines starting with `@`): one command per line, `key=value` output, ends with `ok` or
  `err <text>`. Wrapped by `tools/pinfinder.py` (`--json`, exit status 1 on error).
- **Browser:** `pinfinder.html` in the [web console](https://gitlab.com/ailife8881/remoteuartbus-online)
  (Web Serial, step-by-step setup wizard; needs `tools/cfgfields.json`).
- **Many identical boards:** set one up, export its configuration, and build one image per board id with the
  web console's `image.html` (firmware + config page), flashed at 0x08000000.

| Command | What it does |
|---|---|
| `info` | chip id, uid, master/slave, VDD, battery, uart |
| `list`, `get <name>`, `set <name> <value\|pin\|none>` | config words (names as in `tools/cfgfields.py`) |
| `save`, `revert`, `defaults` | write to flash (checks the pins against the main firmware's peripherals) / reload / defaults |
| `detect hall [mV]` | turns the field open loop; finds the hall pins, order A/B/C, inversion and speed timer |
| `detect vbat [V\|mV]` | battery pin and divider (`vbatgain`) from the measured battery voltage |
| `detect current [mV] [shunt mOhm]` | phase current pins, phase and sign; bus current pin, divider and zero |
| `calibrate hall [mV]` | hall edge angles `hallang1..6` |
| `calibrate poles [ms]` | turn the wheel once by hand → pole pairs (`windings`) |
| `calibrate rl [mV]` | phase resistance and inductance → `motorr`, `phasel` |
| `calibrate kv block\|sine [max]` | speed constants for block / sine speed models |
| `calibrate imu` | IMU mounting and level |
| `detect imu` | bit-banged I2C scan of free pins: MPU-6xxx, ICM-2xxxx/4xxxx, BMI160/270, LSM6DS*, QMI8658, … |
| `detect button`, `detect charger`, `detect ir`, `detect latch` | power button, charger, foot sensors, power latch |
| `adc`, `hall`, `pin …`, `step …`, `scan …`, `rotate …`, `poweroff` | diagnostics |

PinFinder works in software (GPIO reads, bit-banged I2C); the main firmware uses the peripherals. PinFinder
therefore warns when a pin works for it but not for the main firmware: halls not on CH1-CH3 of TIM2/TIM3, IMU
not on I2C1 SDA/SCL, passive buzzer without a TIM14/16/17 output.

Without a valid configuration the main firmware does not drive the motor: it blinks every pin (LEDs flash, an
active buzzer beeps) and keeps a known power latch held.

## Controlling the motor

Two protocols on the same UART (config `baud` in units of 100: `1152` = 115200 baud):

**ODrive CANSimple over UART** — ODrive's CANSimple messages and units (float32) in a CRC-protected frame,
polled per board (node id 0..62, 63 = broadcast). Set points per control mode, encoder / bus / Iq / torque /
power feedback, heartbeat with ODrive error bits, Estop / Clear_Errors, and every configuration word as a
parameter (RxSdo/TxSdo, ids in `tools/endpoints.json`). Arming as in ODrive: `Set_Axis_State` 8
(CLOSED_LOOP_CONTROL) arms, 1 (IDLE) switches the gates off; a disarming error leaves the axis IDLE until
Clear_Errors. Host libraries: `tools/cansimple_uart.py` (Python), the web console's `protocol.js`.

**RemoteUartBus v1** (frames start with `/`) — the original Gen2.x protocol, unchanged
([PROTOCOL.md](https://gitlab.com/ailife8881/remoteuartbus-online/-/blob/main/PROTOCOL.md)). State bit 64
disarms, bits 1-32 set the LEDs. ESP32 example:
[hoverboard_hack_esp32_manualspeed](https://github.com/reed-dan/hoverboard_hack_esp32_manualspeed).

**Several boards on one bus:** master TX to every board's RX, every board's TX to the master RX. Boards drive
TX open drain, so the master RX needs a pull-up (FT232R adapters have one; ESP32: `pinMode(RX, INPUT_PULLUP)`
after `Serial.begin()`); 115200 baud is reliable. A single board can use push-pull TX (`txpushpull=2`, tested
to 921600 baud) — never with several boards on one line.

## Drive modes

`drivemode` = commutation × 4 + control (control 0-3 as the GD32 firmware's iDrivingMode):

| | pwm (0), ‰ | speed (1), rpm | torque (2), 0.01 A | position (3), hall steps |
|---|---|---|---|---|
| block (0) | 0 | 1 | 2 | 3 |
| sine (1) | 4 | 5 | 6 | 7 |
| FOC (2) | 8 | 9 | 10 | 11 |

- **Command 0:** pwm modes coast; speed modes brake and hold; torque modes give zero current (FOC) or coast;
  position modes hold the target. When the remote goes silent the board holds where it is.
- **Block:** high side chops, the other phase's low side stays on, so the current freewheels — the wheel
  turns from ~5 % duty, speed linear in duty. Torque from a current loop on the bus shunt.
- **Sine:** no current sensing needed; torque from `motorr` and the back EMF. Bus current is measured inside
  the active vectors.
- **FOC:** needs two phase current pins (`iphasea`, `iphaseb`, `iphasecfg`). Position mode in 1/256 hall step
  with a stepper-like hold at the target (`hold_current`, smart idle hold `holdidle`) that carries a load.
- **Field weakening** (off by default): FOC `fwimax` (negative d current, capped per rpm for stability),
  sine `fwadvance` (phase advance).
- **Regenerative braking:** with nowhere for the energy to go (bus above `batfull` + 0.5 V) the motor is
  shorted with a ramped, current-limited duty instead of pumping the supply.
- **PWM frequency:** `pwmres` 2048..16383 = 17.6..2.2 kHz (default 8192 = 4.39 kHz); the control loop always
  runs at ~4.39 kHz. Higher frequencies lower the FOC modulation limit (94 % at 4.39 kHz, 76 % at 17.6 kHz).
- **Dead time:** `deadtime` in ns (default 500).

## Settings

Every setting is a CANSimple endpoint with **ODrive's name and unit** where ODrive has one
(`tools/endpoints.json`): `vel_limit`, `vel_ramp_rate`, `torque_ramp_rate`, `current_soft_max`,
`current_hard_max`, `pos_gain`, `vel_gain`, `vel_integrator_gain`, `current_control_bandwidth`,
`phase_inductance`, `inertia`, `enable_overspeed_error`, `enable_torque_mode_vel_limit`, input modes
(passthrough, VEL_RAMP, TORQUE_RAMP, POS_FILTER, TRAP_TRAJ) and the `hoverboard.*` extras (position hold,
field weakening, IMU, charger, foot sensors).

Current limits, always in this order: `current_soft_max` (control loops) < `curpull` (pull back, no error) <
`current_hard_max` (trip). Config words are range checked; a stored value out of range falls back to its
default at boot. A config save switches the gates off while the flash is written.

Command line access over SWD: `python3 tools/mm32config.py show` / `set <name> <value>`.

**Stand-alone options** (off by default; for boards without a host that watches battery and temperature):
`autooff=<minutes>` (power off when idle), `batdeadoff=2` (power off at the low battery cut-off),
`tempderate=<°C>` (linear derating below `templimit`).

## Peripherals: IMU, buzzer, charger, foot sensors

- **IMU** (`imusda`, `imuscl`, `imuaddr` from `detect imu`): read by I2C1 + DMA every 10 ms. Register maps:
  InvenSense MPU-6050/6500/9250, ICM-206xx (tested), ICM-42605/42688, BMI160, LSM6DS3/DSO, QMI8658
  (untested). CANSimple parameters 0x150-0x159: state, accel (m/s²), gyro (°/s), temperature, **pitch and
  roll** (complementary filter; after `calibrate imu` in ROS board axes: X forward, Y left, Z up).
- **Buzzer:** active buzzer by default; `buzzerpassive=2` drives a 2.2 kHz square wave from a TIM14/16/17
  output.
- **Charger** (`charge`, `chargelevel`, `chargepull`): reported in heartbeat byte 7 bit 0 and
  `hoverboard.charger_connected`; `chargelock=2` keeps the gates off while charging.
- **Foot pad sensors** (`ir1`, `ir2`, `irmode`): reported as `hoverboard.foot_sensors` and heartbeat byte 7
  bits 1-2.

## Safety: watchdog, faults, current and voltage limits

- Independent watchdog ~0.5 s; any reset releases the power latch, so a hung board switches itself off.
- A fault or unexpected interrupt switches the bridge off (TIM1 MOE) before the watchdog resets.
- Hardware current limit: ADC watchdog and the break input (comparator) cut the gates for 2 control periods
  per event; repeated break events raise DRV_FAULT and disarm.
- Overvoltage: the board stops feeding energy into the bus above `batfull`. **A bench supply cannot sink
  current** — set `batfull` near its voltage and ramp down instead of coasting at high speed.
- **Debugging:** a halt longer than 0.5 s resets the chip; set DBGMCU_CR bit 8 (0x40013404) first. A halted
  core leaves TIM1 switching: clear TIM1_BDTR bit 15 (0x40012C44). `mm32flash.py` and `mm32config.py` do
  both. To build without the watchdog remove `#define WATCHDOG` in `Inc/pinout.h`.

## Repository layout

```
Makefile                          forwards to HoverBoardMindMotion/Makefile
HoverBoardMindMotion/
  Makefile                        GCC build: MCU=, pinfinder, targets, release, flash
  Src/, Inc/                      main firmware (FOC, sine, block, CANSimple, RemoteUartBus, IMU, config)
  PinFinder/                      PinFinder firmware (shares hardware, config and uart code with Src/)
  Device/, Library/               MindMotion register headers, CMSIS, HAL, startup, linker script
tools/
  mm32flash.py                    SWD flasher (pyOCD), keeps the board configuration
  mm32config.py, mm32cfgstore.py  read / write the configuration over SWD
  pinfinder.py                    PinFinder agent-mode client
  cansimple_uart.py               CANSimple over UART host library
  cfgfields.py, gen_*.py          config field table → cfgnames.h, cfgfields.json, endpoints.json
  pfstrings.py                    PinFinder message compression (build step)
extraFiles/                       pictures
```

## FAQ

**Which hoverboards does this firmware support?**
Split Gen2.x hoverboards whose mainboards carry a MindMotion MM32SPIN05, MM32SPIN06, MM32SPIN25 or MM32F031
MCU. For GD32 / STM32 boards use the
[Gen2.x GD32 firmware](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x).

**Do I need to recompile for my board layout?**
No. Build one image per MCU; PinFinder detects the pins and stores them in flash.

**Does it do FOC?**
Yes, with two measured phase currents (internal op-amps on the MM32SPIN05PFOP, external amplifiers on other
boards). Block and sine work without phase current sensing.

**Can I use it with ODrive tools or code?**
The messages, units and parameter names follow ODrive's CANSimple, carried over UART instead of CAN.
`tools/cansimple_uart.py` is a Python host library.

**Can I control it from an ESP32 or Arduino?**
Yes, over the UART with either protocol; several boards can share one bus.

**Which programmer do I need?**
Any SWD probe supported by pyOCD: ST-Link V2 clone, DAPLink, J-Link.

**Is there a precompiled binary?**
Binaries are not stored in git. Run `make release` or download them from the project's releases.

## Support and contributing

- Discord: https://discord.gg/dvrKS9uyV7
- Matrix: https://matrix.to/#/#hoverboard-firmware-hack:matrix.ai-life.xyz
- Feature requests and bug reports: open an issue or a
  [discussion](https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32/discussions). Please ask
  before opening a pull request — the project changes quickly and PRs get out of sync.

Keywords: hoverboard firmware, hoverboard hack, MM32SPIN05, MM32SPIN06, MM32SPIN25, MM32F031, MindMotion,
BLDC motor controller, FOC, field oriented control, sine commutation, hub motor, robot wheel, ODrive
CANSimple, UART, open source, GCC, pyOCD, PinFinder.
