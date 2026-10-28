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
make                                   # build/hoverboard.bin, .hex, .elf (MM32SPIN05)
make MCU=F031                          # another MCU: SPIN05, SPIN06, SPIN25, F031 -> build/<mcu>/
make targets                           # main firmware + PinFinder for every MCU (compile check)
make CROSS=/path/to/bin/arm-none-eabi- # toolchain not on PATH
make TRACE=1                           # with the FOC capture buffer for SWD tools (768 bytes of RAM)
```
Needs `arm-none-eabi-gcc` (tested with 14.2.Rel1) and `make`. The device headers (MindMotion
register headers, CMSIS) are in `HoverBoardMindMotion/Device/`. A change of the flags (`TRACE`,
`CROSS`) rebuilds everything.

Integer division runs on the chip's hardware divider (`Src/hwdiv.S`) and floats on a compact soft
float (`Src/softfloat.c`, round to nearest even like libgcc, subnormals read as 0); both replace
libgcc's routines. Size: 29.1 of 30 KB flash, 1.4 KB of 4 KB RAM plus the 1 KB stack reserve (the
worst case stack, main loop plus nested interrupts, is ~670 bytes).

MCUs (`make MCU=...`, `Inc/mcu.h`). All four share the SPIN05 register headers and HAL: their
timers, ADC, UART, DMA, GPIO, RCC, flash, watchdog and hardware divider have the same addresses and
register layout (compared in the vendor SVD files). Every image uses the 32 KB layout (30 KB code, the
config in the last 2 KB) and 4 KB of RAM, so the tools work the same on all of them.

| MCU | state | differences |
|---|---|---|
| **MM32SPIN05** | tested on hardware (bench board) | reference; internal op-amps (PFOP) need no setup |
| MM32F031 ("q") | PinFinder runs on a board (UART PB6/PB7 found, nothing driven yet) | the same die as the SPIN05 (IDCODE 0xCC4460B1) without the gate driver; no UART2 pins |
| MM32SPIN06 | builds, untested | the SPIN05 pin tables (as the old Keil target) |
| MM32SPIN25 | builds, untested | pin/AF tables from its datasheet; 72 MHz from the PLL (the old Keil build ran 96 MHz); op-amps OP1-3 (outputs PA6, PB2, PB10) enabled when their output pin is a configured current pin |

The PWM pins are PA8-PA10 / PB13-PB15 (AF2) on all of them; a board that wires its gate driver
elsewhere is not supported.

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
- A halted core leaves TIM1 running: the bridge keeps switching with the last compare values (a static
  voltage vector, against the back emf at speed). The tools' halt (`Mm32.halt()`, used by
  `mm32flash.py` and `mm32config.py`) clears TIM1 MOE / AOE right after the halt and its resume sets them
  back (measured: the hold current's switching stopped while halted, the hold went on after the resume).
  A plain debugger halt does not: clear bit 15 of TIM1_BDTR (0x40012C44) first. DBGMCU's TIMx_STOP
  bits only freeze the counter.

## Control methods
Two protocols on the same UART (one baud, config field `baud` in units of 100: 1152 = 115200):

- **CANSimple over UART** (firmware 2.0): ODrive's CANSimple messages and units (turns, turns/s, Nm, A,
  V; float32) in a CRC-protected UART frame, polled per board. It covers set points per control mode,
  encoder/bus/I_q/torque/power feedback, heartbeat with ODrive error bits, Estop/Clear_Errors, and every
  configuration word and runtime value as a parameter (RxSdo/TxSdo, ids in `tools/endpoints.json`).
  Host libraries: `tools/cansimple_uart.py` (Python) and the web console's `protocol.js`.
  Arming as in ODrive: Set_Axis_State CLOSED_LOOP_CONTROL (8) arms, IDLE (1) switches the gates off.
  An error that disarms (Estop, CURRENT_LIMIT_VIOLATION, VELOCITY_LIMIT_VIOLATION) leaves the axis IDLE:
  after Clear_Errors the master arms again. A Set_Axis_State also hands the gates to CANSimple; until
  then (boot) they follow the v1 state bit.
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
- **Block output:** in each commutation step the high side chops and the other phase's low side stays on
  (`Inc/commutation.h`), so the current freewheels continuously: the wheel turns from ~5 % duty and the
  speed is linear in the duty (bench: 0.87 rpm per permille above 5 %). Before, both chopped together and
  the current collapsed in every off time (nothing below 20 % duty). The block speed loop measures on the
  last hall interval below 60 rpm (like FOC): steady from 15 rpm, steps overshoot <= 2 %.
- **Torque:**
  - FOC: measured phase current.
  - Block: a current loop on the bus shunt, which carries the phase current at the sample time in
    block mode.
  - Sine: no current sensing; the voltage is computed from `motorr` and the back EMF.
- **Bus current in sine without phase current pins:** the shunt is sampled inside the active vectors (the adc
  trigger moves to TIM1 CC4 between the compare values for one sample per control period, the two windows
  alternating) and weighted by their durations; the zero vector sample reads ~0. It follows the load on the
  bench (braked by hand), the scale depends on the shunt calibration (`itotaldivider`, `itotalzero`).
- **Field weakening** is off by default (CANSimple: `hoverboard.foc.field_weakening_current`,
  `hoverboard.sine.field_weakening_advance`; the console's drive panel has both).
  - FOC: `fwimax`, the most negative d current, at most 90 % of the current limit. A controller sets
    the d current so the current loops keep 5 % voltage headroom. Each amp of d current saves
    omega*L of voltage (~0.37 V at 600 rpm on the bench motor, L ~0.46 mH). Bench wheel at 24 V with
    `current_soft_max` 15 A, 94 % modulation: 544 rpm without, 644 rpm with 5 A (0.4-0.7 m/s^2 unloaded,
    also in partial field weakening and through steps in and out of it). The d current is also limited
    to 8 mA per rpm (`FW_MA_PER_RPM`, foc.c): a negative d current in a frame that is off the rotor makes
    torque I_d sin(delta) that pushes it further off (a negative spring; a positive d current is the
    stable spring of the position hold), and the hall frame follows a rotor wobble better the faster it
    turns. Bench, forced d current at 36 V (no field weakening needed): -4.5 A shook at 300 rpm (3.1
    m/s^2), not at 450-750; -6 A shook at 600 rpm (3.9), not at 750; 0 A nowhere (0.55-0.97 m/s^2 at
    300-800 rpm). Uncapped, 9 A of field weakening at 24 V reached 680-733 rpm but shook (2.8 m/s^2);
    capped, any `fwimax` stays smooth (0.6-0.9) and tops out at ~647 rpm at 24 V. A higher bus voltage
    moves base speed up (~830 rpm at 36 V), where the cap allows more.
  - Sine: `fwadvance`, phase advance in degrees. Sine field weakening has no current control and is
    expensive; see `Inc/pinout.h`.
  - Above the speed the battery voltage allows, the back EMF is higher than the bus: a coast (gates
    off) then charges the bus through the MOSFET diodes, which a bench supply cannot take. Ramp down
    (VEL_RAMP; voltage commands slew at 1 ‰/ms) instead of stopping at once.
- **Overvoltage:** the board coasts while the bus is 3 V above `batfull`, so braking doesn't pump a
  bench supply.
- **FOC position:** positions are kept in 1/256 hall step (90 steps per turn). Far from the target a
  P loop on the fine position (`pos_gain`, default 20 (turns/s)/turn like ODrive) feeds the speed loop,
  limited to the speed it can stop from at 20 turns/s^2 and accelerating at that rate. Within 1.5 steps
  and below 25 rpm the hold
  takes over: the current vector is put at the target's electrical angle (d current 40 % of the
  current limit, at most 4 A) and the rotor is pulled there like a microstepped stepper. The q axis
  carries the load: it starts with the speed loop's integral (the current that held the load on the
  way in) and adds to it while the hall sensors show the rotor sagging more than ~1 deg; it is voltage
  driven (`motorr` x current), which also damps the rotor. Leaving the hold hands that current back to
  the speed loop, so a load does not drop. Bench wheel: 1-5 steps settle in ~0.05 s, half a turn in 0.2-0.35 s;
  the rotor stands within 0.25-0.5 deg of the target (hall edge check, unloaded). A load above the
  hold torque makes it slip; the position loop then takes over again. Sine/block position modes work
  in whole hall steps. Position mode runs the speed loop on the last hall interval with 10/3 of the
  velocity gains below 60 rpm, falling to 1x at 200 rpm (higher at cruise speed was unstable). On a
  bench supply set `batfull` near its voltage: hard stops regenerate (the bus rose to 42 V).
- **VEL_RAMP** (`vel_ramp_rate`): the set point runs at most 0.25 s of the ramp (at least 30 rpm)
  ahead of the wheel, so an unreachable command doesn't wind it up: a lower command takes effect at
  once. The command is limited to `vel_limit` before the ramp.
- **PWM frequency:** `pwmres` (TIM1 period, read at boot), 2048..16383 = 17.6..2.2 kHz; default 8192
  (4.39 kHz). The control (the adc interrupt: 88 / 110 us average / max in FOC, 53 / 73 in sine) always
  runs at ~4.39 kHz: below 8192 every n-th pwm period (4096: 2 = 8.8 kHz, 2730: 3 = 13.2 kHz, 2048: 4 =
  17.6 kHz), by TIM1's repetition counter and the adc on its TRGO, so the outputs of a control step hold
  for its whole period. (Before, the interrupt ran every period and overran it from ~8.8 kHz: the sine
  angle stood still while the field kept driving, up to ~100 W into a standing motor.) Cost: the phase
  current op-amps need 2.5 us of low side time either side of the sample, so the FOC modulation limit
  falls with the period: 94 % at 4.39 kHz (top speed 544 rpm on the bench wheel at 24 V), 88 % at 8.8
  (509), 84 % at 13.2 (472), 76 % at 17.6 kHz (436); field weakening makes up the rest. Block and sine
  run unchanged at every setting (bench: the same currents at 0.25-2 turns/s, ~1 W from the supply).
  17.6 kHz is above most people's hearing.

Configs saved before the version field (word 63) used 2/3 for SINE_VOLT/SINE_SPEED; they are
migrated at boot. FOC needs the two phase current pins (`iphasea`/`iphaseb`, `iphasecfg` selects the
phases and polarity).

### Settings (ODrive names)
All settings are CANSimple endpoints with ODrive's names and units where ODrive has the setting
(`tools/endpoints.json`, the web console's parameter table). Besides those that existed: `torque_ramp_rate`
(input mode 6 TORQUE_RAMP), `inertia` (torque feed forward of the TRAP_TRAJ / POS_FILTER acceleration),
`enable_overspeed_error` / `vel_limit_tolerance` (VELOCITY_LIMIT_VIOLATION), `enable_torque_mode_vel_limit`
(default on as in ODrive: torque control stops accelerating at `vel_limit`), `phase_inductance`,
`current_control_bandwidth` (sets the current loop gains kp = L*bw, ki = R*bw), and for the FOC position
hold `hoverboard.position.hold_current` (stiffness), `hold_enable`, `hold_window`, `accel_limit`.
Every loop's gains are settings: position P (`pos_gain`), I (`hoverboard.controller.pos_integrator_gain`,
within 4 hall steps of the target) and D (`pos_derivative_gain`, on trajectory speed - wheel speed);
velocity P / I (`vel_gain`, `vel_integrator_gain`; sine / block: config words `sinekp`/`sineki`,
`speedkp`/`speedki`) and D (`hoverboard.controller.vel_derivative_gain`; `sinekd`, `speedkd`) on the
measured acceleration (20 ms filter); the FOC current loop's P / I (`curkp`, `curki`, or
`current_control_bandwidth`). The I and D terms default to 0 (off). Measured on the bench wheel: sine D 3
lowered a 150 rpm step's overshoot from 5 to 2 %; FOC velocity D raised it (3 -> 8 -> 13 % at 0.02 / 0.05
Nm/(turns/s^2): the lagging hall speed and the integrator); position I took a hold-off move that stopped
a step short to the target (0.24 s at 60).
Position tuning on the bench wheel (FOC, hold off, steps of 1 hall step to 2 turns): the overshoot of long
moves (32 steps on 2 turns) was the regeneration limit, not the gains: braking pumped the bench supply to
`batfull` (27 V) and the braking current was cut. Approach `accel_limit` 5 turns/s^2 stays below it (<= 1
step); 20 is fine on a battery. `pos_gain` 10 overshot fewer steps than 20 (0-3 vs 4-7 of 12, all <= 1
step) at 0.1 s more for 5 steps; lower velocity integrator gains hunted. With the hold on the same moves
settle in 0.03-0.3 s without overshoot.
Tuned with the hold on (passthrough, bench supply): the gains hardly matter (pos_gain 20..60, hold_window
1..2.5, vel_gain x0.75..1.5 all within 0.02 s); the approach `accel_limit` sets the pace: 7 turns/s^2
is the fastest below the regeneration limit (1 step 0.04 s, 5 steps 0.16 s, half a turn 0.51 s, 2 turns
1.00 s, no overshoot). `hold_current` 4 A holds within 0.25-0.62 deg at 6.7 W from the supply, 6 A within
0.12-0.38 deg at 15.3 W. Smart hold (`holdidle`, 0.1 A, off by default; `holdidledelay`, ms, default
500): after that long in the target's hall sector the hold current falls to `holdidle` (~4.4 A/s); pushed
off, it rises at once with the displacement, `hold_current` at 1.5 steps, where the position loop takes
over (bench, 0.5 A: 4 A -> 0.5 A 0.5 s after arriving, the position exact; nudged a step: 4 A while it
follows, 0.5 A again 1.3 s later). A steady load stays carried by the hold's q current. POS_FILTER is slower and overshoots long moves at every bandwidth (2: 2 turns
not there in 3.5 s; 4: 2.1 s, 4 steps; 12: 1.2 s, 5 steps): passthrough or TRAP_TRAJ for discrete
targets, POS_FILTER for streamed set points.
They are stored in an extension config record (words 64-127) that is saved in the slot before the main
record: PinFinder keeps reading and writing the main record; a record it appends leaves the extension at
the defaults. `tools/mm32config.py show/set` handles both.

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
### Buzzer, charger and foot sensors
* `buzzerpassive=2` for a buzzer without its own oscillator (a plain piezo or magnetic buzzer stays
  silent on a steady level): it gets a 2.2 kHz square wave (the low battery alarm from the adc
  interrupt, the melodies by busy waiting). Default: an active buzzer, switched on and off.
* `charge=<pin>` (+ `chargelevel` 1 low / 2 high while connected, `chargepull` 1 none / 2 up / 3 down;
  PinFinder's `detect charger` sets all three): the charger state, debounced 50 ms, is reported over
  CANSimple in heartbeat byte 7 (bit 0; ODrive leaves the byte unused) and as the parameter
  `hoverboard.charger_connected` (0x114). The v1 reply has no free field, so v1 masters do not see it.
  `chargelock=2` keeps the gates off while the charger is connected.
* `ir1`, `ir2` (+ `irmode`, PinFinder's `detect ir`): the infrared foot pad sensors, debounced 50 ms,
  reported as `hoverboard.foot_sensors` (0x115, bit 0 / 1 = sensor covered) and in heartbeat byte 7
  bits 1-2. Reported only: nothing in the drive logic depends on them. Tested without sensors (pins
  pulled up / down over SWD); no board with them here.
### IMU
* The IMU found by PinFinder's `detect imu` (`imusda`, `imuscl`, `imuaddr`) is read when it has the
  InvenSense register map: address 0x68/0x69 (MPU-6050/6500/9250, ICM-206xx and clones; not
  ICM-426xx or BMI160/270, which also sit at 0x68). Bit-banged I2C on any two pins (`Src/i2c.c`),
  every 10 ms, one byte per main loop pass (~60 us each), +-4 g and +-1000 deg/s, 44 Hz low pass; a
  chip that resets is set up again within a second.
* CANSimple parameters 0x150-0x159: `hoverboard.imu.state` (0 none, 1 running, 2 no answer),
  `accel_x/y/z` in m/s^2, `gyro_x/y/z` in deg/s (bias: the mean of the first 0.5 s standing still),
  `temperature` in C, `pitch` and `roll` in degrees. After PinFinder's `calibrate imu` the values are in
  the board's axes (ROS: X forward, Y left, Z up) and pitch / roll are zero at the calibrated level;
  pitch is positive with the front down, roll with the right side down. Complementary filter: the gyro
  integrated, pulled to the accelerometer's angle with a ~0.5 s time constant. Without the calibration
  the chip's axes. Bench check: front down 52 deg, right side down 34 deg, back to 0.0.
* Bench board: a clone at 0x69 on PD0/PD1 that answers WHO_AM_I with 0x2E instead of 0x68 but has the
  MPU-6050 register map.

## Autodetect&configuration (PinFinder)

PinFinder is a separate firmware that finds the board's pins and writes the configuration the main
firmware boots from. Build and flash it from this tree, then flash the main firmware again (both keep
the configuration page):
```bash
cd HoverBoardMindMotion
make pinfinder && make flash-pinfinder   # build/pinfinder.bin
make flash                               # back to the main firmware
```
It talks 115200 8N1 on the configured TX/RX pins. On a board without a configuration the uart pins
are searched first: connect the board's TX and RX together (loop-back) until it finds them, then
connect your adapter. It prints `PinFinder 2.0: Enter = menu, @<command> = agent mode` every 2 s.

**TUI** (Enter): the pin map (unsaved changes in yellow), battery and supply voltage and a menu: hall
sensors, LEDs/buzzer (step through the free pins, the tried one blinks; `t` plays a tone for a buzzer
without oscillator), battery pin, power button, charger, power latch, slave id, test motor, power off,
`a` all, `c` command line (the commands below), `s` save. Without an ANSI terminal the same text is
printed without colours.

**Agent mode** (a line starting with `@`, for scripts and agents; stays on, `tui` goes back): one
command per line, no prompts or colours. PinFinder echoes `> <command>`, then prints `key=value`
lines, `# notes`, `warn ...`, `progress n/m`, and ends with `ok` or `err <text>`. Any byte received
during a long command stops it. `tools/pinfinder.py` wraps this (`--json`, exit status 1 on `err`):
```bash
python3 tools/pinfinder.py info "detect hall" "detect vbat 36500" save
```
| command | |
|---|---|
| `info` | chip id, uid, master/slave (latch), debugger attached, VDD, battery, uart |
| `list`, `get <name>`, `set <name> <value\|pin\|none>` | config words, names as `tools/mm32config.py` (`tools/cfgfields.py`) |
| `save`, `revert`, `defaults` | write to flash / reload / start from the defaults (RAM) |
| `detect hall [mV]` | turns the field open loop in 12 directions per electrical turn (complementary PWM, 1 V phase amplitude, raised if the rotor does not follow); the pins whose level follows the rotor are the halls, A/B/C is the assignment for which the main firmware's commutation leads the rotor by 60..120 deg everywhere |
| `detect vbat [V\|mV]` | battery pin: the one closest to the measured voltage (36.5 or 36500; without: the only one reading 20..45 V), and with the voltage the divider in 0.001 (`vbatgain`); on failure the old pin stays (`vbat_kept`) |
| `detect current [mV] [shunt mOhm]` | phase current pins, their phase and sign (`iphasea/b`, `iphasecfg`: correlation with each phase's current while a static field turns), then the bus current pin and `itotaldivider` / `itotalzero` (sampled in the middle of an active vector against the phase currents: the pin that rises steadily over 8 levels). With the shunt value from its marking (R003 = 3 mOhm) the divider is exact and `iphasegain` is calibrated from it |
| `calibrate hall [mV]` | hall edge angles: the field turns 2 electrical turns forward and back in 1 deg steps, each edge is the mean of both directions -> `hallang1..6` (the main firmware uses them instead of its built-in table, +-30 deg around fixed centres) |
| `calibrate poles [ms]` | turn the wheel one turn by hand: hall edges / 3 -> `windings` |
| `calibrate rl [mV]` | phase resistance (static field at two voltages against the phase currents) and inductance (current rise after a voltage step, sampled every pwm period) -> `motorr`, `phasel` |
| `calibrate kv block\|sine [max]` | free-running speed at 3 levels: block `speedkv` / `speedd0` (the COM_SPEED model rpm = kv x Vbat x (duty - d0)), sine `sinekv` / `sined0` (field 90 deg ahead of the hall state; the torque constant follows from `sinekv`) |
| `calibrate imu` | the IMU's mounting: the board stands still as when driving (up, and the level), then is tilted forward by 20-60 deg and held (gravity moves towards -forward) -> `imumount` (1 + up*6 + forward, chip axes 0..5 = +x +y +z -x -y -z), `imurestx/y/z` (gravity at rest: zero pitch and roll) |
| `detect imu` | free pins with an external pull-up, every pair tried as bit-banged I2C; the address and id register (MPU-6xxx incl. the 0x2E clone, ICM-2xxxx/4xxxx, BMI160/270, LSM6DS*, QMI8658, LIS3DH, SC7A20, QMA*; an unknown chip at an IMU address is kept as `unknown`) -> `imusda`, `imuscl`, `imuaddr`, `imuid` |
| `detect button [ms]`, `detect charger [ms]` | the pin that changes while you press the button / connect the charger (pins read with pull-up and pull-down: driven 0/1 or floating) |
| `detect ir [ms]` | the two infrared foot sensors: cover one, then the other when asked; per sensor the pin that changes, its level while covered and its input pull (as `detect charger`) -> `ir1`, `ir2`, `irmode` |
| `detect latch` | releases the free pins one by one, saving each as the latch first: the board switches off at the latch pin, the next start holds it |
| `adc`, `hall` | voltages of the free adc pins, hall state now |
| `pin <pin> high\|low\|pullup\|pulldown\|input\|read\|blink\|tone [ms]` | drive a free pin (LEDs, buzzer); `read` works on any pin: level, voltage (adc pins), what it is used as |
| `step <1-6\|101-112> [mV] [ms]` | hold a block step or field direction; hall state and the currents (triggered like the main firmware) |
| `scan <k> <mV> <pin>` | hold field direction k (0-11) and sample a pin at 32 points of the pwm period |
| `rotate <permille> [ms]` | hall commutation test as the main firmware does it; counts forward/backward hall steps |
| `poweroff` | |

A browser page for the same, over Web Serial: `pinfinder.html` in the
[web console](https://gitlab.com/ailife8881/remoteuartbus-online) (needs `tools/cfgfields.json`, written by
`tools/gen_cfgnames.py`, next to it).

The old PinFinder (Keil, `pin-finder` branch) is described in the
[wiki](https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32/-/wikis/home/%7BHow-to-use-PinFinder-firmware%7D).

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






