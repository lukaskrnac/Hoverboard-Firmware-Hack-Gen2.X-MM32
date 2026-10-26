# TODO and progress

State of the `gcc-build` branch (local, not pushed). Bench setup: MM32SPIN05 board on a 24 V bench
supply, unloaded wheel, ST-Link for SWD, FT232R USB-UART on the masterslave header (TX PB6 / RX PB4).

## Next

- [ ] **Hall angle calibration in PinFinder.** The sine angle estimator uses `CWAngleTab` (hall state
      centres) and assumes the sector boundaries at ±30° (`hallhandle.c`). That was measured on one
      bench motor; other motors or hall placements can differ. Add a PinFinder menu item that:
      1. puts TIM1 in sine output mode, applies a small d-axis vector (Ds only, ~5 % of 32768) at a
         known angle θ with the firmware's SVPWM (`PWM_GEN_calc`), so the rotor aligns to θ,
      2. sweeps θ slowly over 2 electrical turns forward and back (256 per step, ~12 ms per step),
      3. logs θ at every hall state change, averages the forward/backward edge (hysteresis ~23° from
         cogging and friction),
      4. stores the 6 sector centres (or one offset) in the config and the firmware uses them instead of
         the hardcoded table. Also reports the direction (hall sequence 2,6,4,5,1,3 for rising θ here).

      Reference: the host-side version that produced the numbers (core halted, SWD):
      `~/.cache/opencode-scratch/mm32-hover/hallcal.py`. Measured boundaries: 29.9, 90.7, 149.0,
      211.0, 269.6, 329.8° (±1.5°). Keep the vector small: the bus current spikes to ~11 A raw.
- [ ] **PinFinder on GCC.** The `pin-finder` branch is Keil-only and needs the same work as main:
      vendored headers/Makefile, and the globals written by the serial interrupt made `volatile`
      (its busy-waits hang under GCC otherwise, e.g. the ANSI terminal check). A scratch build with
      those fixes works (`~/.cache/opencode-scratch/mm32-hover/pinfinder/`). It has no float printf,
      so the battery-voltage menu prints blank values: print in mV with integers instead.
      Its power-latch detection gives wrong results when the MCU is powered from the debugger.
- [ ] **Current sense calibration.** `itotaldivider` 250 assumes a 4 mΩ shunt (PA7, direct). Compare
      the telemetry current with the bench supply under load and correct it (webcam on /dev/video0
      reads the PSU; the user brakes the wheel by hand). One ADC count is ~0.3 A. The zero offset is
      the mean of 128 samples at boot, biased by switching spikes, and the ADC clips at 0 (no negative
      current): use a median / exclude spikes. In sine mode the shunt is sampled while all low sides
      conduct and carries ~0 there: it needs another sample time (FOC computes the bus current from
      the phase currents and duties instead).
- [ ] Web console: drive mode numbers changed (sine 2/3 -> 4/5, now 0-11); its config frame still
      sends the old numbers, which now mean block torque/position (2/3).
- [ ] Position: FOC settles 1-2 steps off (the speed set point near the target is below what the
      halls can measure); the target is int16 (the command field). Sine takes 1.6-2.2 s per move.
- [ ] Sine torque is a model (MOTOR_R, the sine kv and dead band): it reads high against FOC at the
      same command; block torque depends on the shunt calibration.
- [ ] FOC: I_d/I_q noise ~±0.3 A at standstill (±5 counts on the op-amp outputs) and ~±1 A at
      500 rpm; the median filter delays the current by one period. Check the angle estimate at speed
      (park angle lead), and filter id/iq for telemetry.
- [ ] **Low-battery cut-off** is disabled on the bench (`batempty` 19.0 V). Set real limits for a
      battery (e.g. 42.0 / 32.0 V for 10S) with the web console or `mm32config.py`.
- [ ] Push `gcc-build` and the web console (`remoteuartbus-online`, branch `console`; pushing to
      `main` deploys GitLab Pages). Try the console once in Firefox (WebSerial add-on, over http).

## Later

- [ ] Speed measurement averages the last 6 hall intervals: it lags during the first electrical turn
      of a fast acceleration (SINE_SPEED steps overshoot up to ~10 %), and one bad sample (178 rpm at
      150) was seen during an acceleration. FOC_SPEED uses `fastspeed` (last interval); the other
      speed modes could too.
- [ ] After a COM_SPEED -> SINE_SPEED switch at speed, one hall sector takes 10-28 ms instead of
      4.4 ms (150 rpm) and the next ones come faster: the speed reading dips for one electrical turn
      (to 120, once 68 rpm) although the ADC ISR keeps running. Probably the stator current transient
      at the switch moving the hall switching points (armature reaction); check with a phase current
      probe, or ramp the sine modulation in over a few ms.
- [ ] Block commutation needs 18.5 % duty before the wheel turns (sine: 1 %); it freewheels and only
      coasts down. Check the switching pattern (one PWM high side + the complementary output of the
      low side) against the gate driver's input polarity; complementary switching would also brake.
- [ ] Host unit tests for the pure logic (CRC and frame layout against the web console's test
      vectors, SpeedCtrl() against the measured plant) and a CI build. Needs the logic split from
      the HAL-dependent code first.
- [ ] Naming is mixed (camelCase, snake_case, Hungarian `iSpeed`/`bConfigRx`, vendor `u8`/`s32`
      next to stdint; config macros like `DRIVEMODE` are lvalues of `pinstorage[]`): clean up file by
      file when a file gets real work, not in one sweep.
- [ ] `UART_GPIO_Init()`: the last matching table entry wins (PB7 is both UART1 RX and UART2 TX):
      select the UART from the TX/RX pair instead of relying on table order.
- [ ] RemoteUartBus: state bit 128 (shut off) and `slaveNew` are not implemented; there is no read
      command for the configuration (the console cannot show the board's drive mode). A position mode
      (GD32 drive mode 3, odometer target) would be the useful new mode.
- [ ] MM32SPIN06 untested with the GCC build; MM32SPIN25 not ported (needs the MM32SPIN2x pack
      headers, see `HoverBoardMindMotion/Device/MM32SPIN2x/Source/README.md`).
- [ ] Unused board hardware on the bench board: charger detect PB10, E-stop PB0, I2C PD0/PD1, WS2812
      headers. (PA0–PA6 are not comparators: the PFOP's op-amps, phase B/C current, see README.)

## Done (this branch)

| commit | |
|---|---|
| 3ba2e41 | GCC build: Makefile, startup, linker script (last 2 KB kept for the config), vendored MindMotion + CMSIS headers |
| 87da85f | ADC ISR integer math (soft-float took 45 % CPU under GCC, broke starting); current offset underflow; unconfigured ADC pins |
| b397052 | globals declared in module headers, include guards (`dir`, `iOdom`, const tables had drifted) |
| 964ed9a | HAL reports the real clock, UART no longer hardcodes 72 MHz |
| 280d10f | `volatile` on state shared with interrupts |
| 00aee25 | Keil project removed; `tools/mm32flash.py`, `make flash` (keeps the config, soft start keeps the board powered) |
| 90a1d28 | `tools/mm32config.py`: read/change the board config over SWD |
| c88adbd | soft current limit in A on the averaged current (was 0.1 A on raw samples) |
| 7dbd062 | config frame applied in the main loop, saved only when stopped; parser fixes |
| e7948f3 | wState disarm (64) and LED bits; gates owned by the ADC ISR (AOEN cleared with MOEN); open-drain TX |
| aee5ae3 | README: protocol, web console, bus wiring |
| 05cc4f6 | hall measurement on real edges only; commutation on direction change; timer overflow |
| fc734f3 | COM_SPEED: PI + feed forward + reference model (was a double integrator) |
| f0547ed | sine: calibrated hall angle estimator, interpolation per real ISR period; SINE_SPEED on the same controller |
| f3ee9ca a960618 | faults/unused vectors switch the gates off; watchdog on (~0.5 s) with bounded init; flasher stops DMA (UART RX DMA wrote into the programming buffer) |
| 3521dd6 | power button shutdown through the gate owner; all waits feed the watchdog |
| fcdbff9 | drive mode change while running: gates off + ISR masked; bumpless speed controller restart |
| a756f4a | tools restore registers, RAM, DMA after a halt (resume after `mm32config set` faulted or ran with garbage) |
| e9dcff6 656b0c3 | -Wextra etc. for our code, prototypes, include guards, scoped pack; dead code removed, file-local symbols static |
| a6a38bf 6473849 6bb2952 775ecd2 | -nostartfiles, Makefile dependency, UTF-8 sources, whitespace/includes, .editorconfig |

Verification for every change: flashed to the bench board and checked with RemoteUartBus telemetry
(bus replies, speed steps, CPU load by IPSR sampling); refactors also by comparing `objdump -d -r` of
the objects before/after.
