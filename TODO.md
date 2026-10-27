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
- [ ] Bus current in sine mode on boards **without** phase current pins: the shunt is sampled at the
      zero vector and reads ~0 (noise); they need a second sample inside an active vector. (With the
      pins, sine now uses the phase currents like FOC, up to 90 % modulation.)
- [ ] Position (v1 protocol): the target is int16 hall steps (the command field). Sine takes 1.6-2.2 s
      per move.
- [ ] Sine torque is a model (MOTOR_R, the sine kv and dead band): it reads high against FOC at the
      same command; block torque depends on the shunt calibration.
- [ ] FOC field weakening with 13 A of d current (~840 rpm at 24 V) shakes and trips (9 A: 0.7-0.9
      m/s^2, stable). The hall edges move with the stator current (intervals +-9 % within a few
      edges at 820 rpm, far more than the rotor can change); the tracker follows them, the frame
      error times |I_d| is torque, which moves the edges again. The tracker gain is already 1/16 per
      edge at speed. Next: measure the edge shift against I_d / I_q at a fixed speed and correct the
      measured boundary for it, or cap `fwimax` where it stays stable. A trip there switches the
      gates off above base speed: the back emf charges the bus through the diodes (a bench supply's
      overvoltage protection cut out). Braking from above base speed also raises a bench supply's
      bus (ramps pause and no braking current above `batfull`).
- [ ] Partial field weakening (target speed just above base speed, the d current between 0 and
      `fwimax`): the speed swings +-8-12 rpm at 1-10 Hz, 1.1-1.3 m/s^2 at 570-600 rpm with 5 A (the
      old firmware the same, 1.3); at full field weakening (target beyond reach) 0.4-0.7, without
      field weakening 0.2-0.5. I_d swings +-0.7 A around the field weakening controller's value. The
      field weakening integrator (on the demanded voltage incl. the P terms) and the speed loop
      interact; a feed forward d current from the speed (and the integrator only trimming) is the
      next thing to try.
- [ ] **PWM whine** (4.39 kHz, audible at low speed). The adc isr takes 47-66 us (soft division on
      the M0), so the pwm period cannot shrink with the control running every period. Plan: pwm at
      2-4x (`pwmres` 4096/2730/2048) with the control every n-th period (the isr returns early in
      between), so the control period and all its timing stay 227.6 us. Cost: the op-amps need >= 2.5
      us of low side time either side of the sample (they lag the shunt by ~5 us, measured with all
      phases at one duty): the modulation limit falls from 90 % to ~85 % at 13 kHz, ~80 % at 17.6 kHz
      (top speed before field weakening), and 2-4x switching losses. The output lead (10/8) must be
      measured again then.
- [ ] Sine drawing a lot of current (seen on the bench supply by the user, conditions unknown). Not
      reproduced on the unloaded wheel: at its top speed (95 % modulation) sine takes 0.4 A at 24 V
      (FOC 0.3 A), accelerations peak at ~3.8 A phase current. The board's readings froze above 90 %
      modulation until now (2.5 A shown); fixed, they are valid to 95.3 %.
- [ ] FOC modulation limit 90 % could be ~94 %: the op-amps read correctly down to +-2.5 us of low
      side time (C), +-1.5 us (B); 94 % leaves +-3.4 us. +4 % top speed before field weakening.
- [ ] At high current and speed (field weakening) the ST-Link and the FTDI drop off USB now and then
      (ground noise through the shared USB ground, the hub resets). Test scripts read over the bus
      there; an isolator on the SWD/UART link would help.
- [ ] **Low-battery cut-off** is disabled on the bench (`batempty` 19.0 V). Set real limits for a
      battery (e.g. 42.0 / 32.0 V for 10S) with the web console or `mm32config.py`.
- [ ] Push `gcc-build` and the web console (`remoteuartbus-online`, branch `console`; pushing to
      `main` deploys GitLab Pages). Try the console once in Firefox (WebSerial add-on, over http).

- [ ] FOC position hold with a real load (a weight): the load current handling (speed loop integral
      at entry, +4.4 A/s while the hall state shows >= 1 deg of sag, back to the speed loop at exit) is
      only tested on the free wheel. Check that a hanging weight neither drops nor oscillates.
- [ ] FOC position hold costs current all the time it holds (up to 4 A of d current, ~7 W): drop it
      after a while at the target (like a stepper's idle current) as far as the load allows; make the
      hold current a parameter (no config word left). Sine/block position still work on whole steps.
- [ ] Position: during a hard reversal at speed the hall angle waits for an edge that comes late; with
      speed gains of 60/200 that once ran the currents away (30 A trip). 100/300 (position default) and
      30/100 passed every run; the tracker should handle a reversal between edges better.
- [ ] TORQUE_RAMP input mode (no config word left; words 28-31, 35, 43 hold the other input modes).
- [ ] Flash: 27.3 of 30 KB with LTO (-flto saved 3 KB). Check before adding more.
- [ ] Bus line above 230400 baud: the open-drain TX needs a stronger pull-up (or push-pull for a
      single board: config option).

## Later

- [ ] Speed measurement averages the last 6 hall intervals: it lags during the first electrical turn
      of a fast acceleration (SINE_SPEED steps overshoot up to ~10 %), and one bad sample (178 rpm at
      150) was seen during an acceleration. FOC_SPEED uses `fastspeed` (last interval) below 60 rpm
      and the 6-interval average above 120 rpm (blended between): on the last interval the sector
      width differences (up to 5 deg el, the same every revolution) became torque steps. With field
      weakening active it goes back to the last interval (the loop swung +-13 rpm on the average
      there, 570 rpm with 5 A); why the average is worse there is not understood yet.
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
