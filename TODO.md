# TODO and progress

State of the `gcc-build` branch (local, not pushed). Bench setup: MM32SPIN05 board on a 24 V bench
supply, unloaded wheel, ST-Link for SWD, FT232R USB-UART on the masterslave header (TX PB6 / RX PB4).

## PinFinder and board support (requested 2026-10-27)

Order: P1 is the base for the rest (P2-P7 extend that build, P4 uses P2's protocol). PinFinder
stays a separate firmware image (the main firmware has 4.6 KB of flash left), built from this tree so
it shares `hardware.c`, the HAL, the config layout (`pinout.h`) and the flasher.

- [x] **P1 PinFinder on GCC, in this tree** (rewritten, not ported: `PinFinder/`, `make pinfinder`,
      `make flash-pinfinder`; shares hardware.c, initialize.c, sim_eeprom.c, uart.c, the commutation tables
      (`Inc/commutation.h`), the default record and the layout migration with the main firmware; 20.6 KB).
      Verified on the bench board: info, adc, battery pin, hall detection (3/3 runs A=PC14 B=PC13 C=PC15),
      test rotation, get/set/revert, TUI screens; the main firmware boots unchanged afterwards (config page
      byte-identical). **Not yet tried on hardware (need a person at the board):** `detect button` (press),
      `detect charger` (plug in), `detect latch` (switches the board off), the LED screen (watch the LEDs),
      `poweroff`, the uart loop-back search (needs an erased config), `save` from PinFinder.
- [x] **P2 Agent mode** (`@` lines, `> cmd` echo, `key=value`, `ok`/`err`) + `tools/pinfinder.py` (`--json`),
      README "Autodetect&configuration (PinFinder)". Long commands stop on any received byte.
- [ ] **P3 TUI**: done: one screen with the pin map (unsaved changes yellow), VDD/battery, menu, LED
      screen, test motor screen, command line = agent commands, plain text without ANSI. Open: per item
      help (`?` shows the command list only), "how found" per pin (detected/manual/default), a screen
      for the remaining config words, detect-all ordering when a step fails.
- [ ] **P4 Web GUI config tool.** A page in the console repo (`remoteuartbus-online`, e.g.
      `pinfinder.html`) over WebSerial using the P2 protocol: pin map table, buttons
      per detection with progress, edit any config word with names/units from the same source as
      `tools/endpoints.json` / `pinout.h`, save, export/import the config as JSON, "flash main firmware"
      hint. Also usable against the main firmware's CANSimple config endpoints for the words both
      share. UI test like `uitest/` (headless chrome, WS bridge).
- [ ] **P5 New detections.**
      - **Phase current pins** (`IPHASEAPIN`/`IPHASEBPIN`/`IPHASE_CFG`/`IPHASE_GAIN`): with the gates on and
        a small static SVPWM vector swept over 360 deg (as `opaprobe.py`), sample every free ADC pin at the
        low-side window; pins whose reading follows a sinusoid of the vector angle are phase currents.
        The phase of each sinusoid gives the phase (A/B/C) and the sign gives bit 2 of `IPHASE_CFG`; the
        amplitude against the bus shunt (calibrated by a locked-rotor step, `blockcal.py`) gives the gain.
        Boards without op-amps/phase shunts: report "none" (FOC not available, sine falls back to the
        bus shunt).
      - **DC (bus) current pin** (`ITOTALPIN`): existing item 4 only finds a pin that moves; add the zero
        offset (`ITOTAL_ZERO`, below adc 0 on the bench board) and the divider (`ITOTAL_DIVIDER`) from a
        locked-rotor block step against the phase currents (or against a known load if there are none).
      - **Done (`detect current`, 2026-10-27):** phase current pins + phase + sign (correlation over 12 static
        field directions, complementary PWM): bench PA2 = B, PA6 = C, falling, iphasecfg 1, 4/4 runs; gain
        from motorr shown for information (529-537 vs 515 calibrated). Bus pin (PA7) and its zero/divider from
        the active vector (one phase high) sampled in its middle, one channel per conversion, against
        I_x = -(I_a + I_b): 8 linear points, 2.35 counts/A, zero -3.5..-4.6 counts -> **itotaldivider ~330,
        itotalzero ~400, while blockcal.py gave 419 / 145**. Both use the op-amps (gain 515) as the
        reference; check which is right against the PSU display (webcam) before trusting either. Boards
        without phase current pins: the bus pin is not searched yet (needs a reference: motorr).
      - **IMU**: scan I2C on the hardware I2C pin pairs (PD0/PD1 on the bench board) and bit-banged on
        other free pin pairs with pull-ups; probe 0x68/0x69 (MPU-6050/6500/9250, ICM-20xxx: WHO_AM_I
        0x75), 0x6A/0x6B (LSM6DS*: 0x0F), BMI160 (0x68, CHIP_ID 0x00 = 0xD1); report bus pins, address,
        chip. Store pins + type in new config words (extension record).
      - **Charger detect pin** (bench: PB10): ask the user to plug/unplug the charger, find the input that
        follows it (pull-up and pull-down tried), store pin + polarity.
      - **Hall angle calibration** (the item below, moved into PinFinder).
- [x] **P6 Buzzer without an internal oscillator**: `buzzerpassive=2` (word 79): low battery alarm toggled
      in the adc isr (2.2 kHz), melodies by busy waiting (`Beep()`); PinFinder LED screen `t` tone / `p`
      passive buzzer, agent `pin <p> tone`. Not tried with a passive buzzer yet (the bench one is active).
      Original plan: Config word: buzzer type
      (active / passive) + tone. Passive: toggle the pin from the ADC ISR (4.39 kHz -> 2.2 kHz tone, no
      timer needed, works on any pin; a divider in the ISR gives lower tones), or a free timer channel
      if the pin has one (TIM3/TIM14/16/17, check which are free: TIM1 PWM, TIM2/TIM3 are used now).
      PinFinder's LED/buzzer detection asks "did you hear it" for both drive types and stores the type.
- [x] **P7 Charger state to the remote**: words 80-82 (`chargelevel`, `chargepull`, `chargelock`), 50 ms
      debounce in the main loop, CANSimple heartbeat byte 7 + `hoverboard.charger_connected` (0x114),
      `chargelock`; PinFinder `detect charger` sets pin/level/pull. v1 reply has no free field: not reported
      there. Open: try with a real charger (bench pin map: PB10), web console display. Original plan:
      Read the charger pin (P5) in the main loop, debounced; report
      it: CANSimple (a bit in the heartbeat's controller/axis flags or a new SDO endpoint
      `hoverboard.charger_connected`, plus `tools/endpoints.json`), the v1 RemoteUartBus reply (a state
      bit), and the web console. Optional policy: refuse to arm while charging (config switch).
- [ ] **P8 More targets.** One source tree, `make MCU=...`:
      - MM32SPIN06: same library as SPIN05, untested with GCC; check the pin table, flash/RAM size,
        linker script, the op-amps (if any) and the hardware divider.
      - MM32SPIN25: needs the MM32SPIN2x pack headers (`Device/MM32SPIN2x/Source/README.md`), its HAL is
        vendored in `Library/MM32SPIN25`; different pin count/AFs, op-amps (`hal_op.c`), flash/RAM size
        and the config page address (to check).
      - MM32F030 / MM32F031 (and the `origin/pinfinder-spin280` branch's MM32SPIN0280): general-purpose
        M0 parts on some boards; vendor headers, clock tree (maximum clock to check: PWM_RES and every
        timing assume 72 MHz), no gate driver/op-amps.
      Runtime: PinFinder reports the chip (DBGMCU IDCODE, flash size register) and refuses a firmware
      built for another MCU; the flasher checks the same before programming.
- [ ] **P9 Still not detected / features to consider** (review, then promote to items):
      - E-stop input (bench PB0) and WS2812 header (PB12): detect + support (E-stop = disarm like the
        CANSimple Estop; WS2812 status LED like the RGB LED).
      - Pole pairs (`WINDINGS`): count hall edges per wheel turn (the user turns the wheel one turn, or
        the motor turns slowly with a mark/camera).
      - Motor R and L (`MOTOR_R`, `PHASE_L`): ODrive-style motor calibration (dc step for R, ac/step
        response for L) with the phase currents; torque constant / kv (`TORQUE_CONST`, `SINE_KV`) from a
        free run at a known modulation.
      - Battery voltage divider (`VBAT_DIVIDER`): the user enters the measured voltage once.
      - Low-side polarity (`INVERT_LOWSIDE`) and the over-current comparator pins (`OCPPIN`/`OCPREFPIN`):
        detect safely (gate driver input states with the bridge unpowered?) or keep manual with a warning.
      - Master/slave UART (the second board link) and the remote UART pins: `finduartloop` exists for
        the debug port; extend to both.
      - Temperature: MCU internal sensor (ADC channel) and an NTC pin if fitted; report + derate.
      - Battery cell count from the measured voltage -> suggested `BAT_FULL`/`BAT_EMPTY`.
      - Headlight / brake light outputs on some boards (detect like the LEDs).
      - Hall inputs inverted / 180 deg off the commutation: PinFinder detects it (no assignment fits, the
        inverted one does) but the firmware cannot use it; a `hallinvert` config bit (hallpos() 7-state)
        would make such motors work.
      - Hall edge positions at 30 deg resolution come out of `detect hall` already (12 directions); the
        hall angle calibration (P5) can refine them.

## Next

- [ ] **Hall angle calibration in PinFinder** (see P5). The sine angle estimator uses `CWAngleTab` (hall state
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
- [ ] **PWM whine** (4.39 kHz, audible at low speed). The adc isr takes 88 us on average, 110 us at
      most in FOC (sine 53 / 73, block 28 / 40; measured at 120 rpm with the hardware divider: it was
      126 / 168 in FOC with libgcc's division), so the pwm period cannot shrink with the control running
      every period. Plan: pwm at
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
- [ ] FOC position hold costs current all the time it holds (`hold_current`, default 4 A, ~7 W): drop it
      after a while at the target (like a stepper's idle current) as far as the load allows. Sine/block
      position still work on whole steps.
- [ ] Position: during a hard reversal at speed the hall angle waits for an edge that comes late; with
      speed gains of 60/200 that once ran the currents away (30 A trip). 100/300 (position default) and
      30/100 passed every run; the tracker should handle a reversal between edges better.
- [ ] Flash 26.4 of 30 KB (3.6 KB free), RAM 1.4 KB + 1 KB stack reserve of 4 KB (worst case stack
      ~670 bytes, GCC's call graph `-fcallgraph-info=su` on the LTO link). Next room if needed: the
      vendor HAL init calls inlined into main (TIM1 / ADC / UART init through structs), `odriveParam()`
      (1.2 KB, a switch per endpoint), the 512 byte sine table.
- [ ] Halting the core over SWD (pyOCD, the tools' `Mm32.halt()`) leaves the bridge switching with the
      last compare values: a static voltage vector, at speed against the back emf. Set DBGMCU TIM1_STOP
      (if this MM32 has it: outputs off while halted) or have `halt()` clear MOE / AOE and restore them.
- [ ] ODrive settings not implemented: encoder bandwidth (the hall tracker's gains are per edge and
      measured, see hallhandle.c), calibration_current / motor calibration, dc_max_positive/negative_current
      (bus current limits; the regeneration limit is by voltage, `batfull`), spinout detection.
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
      coasts down. **Cause measured (2026-10-27, PinFinder `step`):** the high side and the other phase's
      low side chop together (CCxNE alone follows OCxREF), so in the off time the current freewheels
      through both body diodes against the bus voltage and collapses: discontinuous conduction. A held
      block step at 8 % duty (2 V) gave 0.46 A, at 25 % (6 V) 1.2 A phase current, against V/R = 3-10 A;
      the same voltage as complementary PWM on all phases gave the expected 2.9 A at 1 V. Fix: keep the
      low side on for the whole step (forced active CCxN, chop only the high side) or switch the chopped
      phase complementary (also brakes).
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
- MM32SPIN06/25, charger, E-stop, I2C/IMU, WS2812: moved to P5-P9 above. (PA0–PA6 on the bench board
  are not comparators: the PFOP's op-amps, phase B/C current, see README.)

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
| 2f4a251 | hardware divider (HWDIV) for `/` and `%`, compact soft float: flash 30.6 -> 26.2 KB, FOC isr 126 -> 88 us |
| 52e423a | RAM -960 bytes: FOC trace only with `make TRACE=1`, int16 averaging samples |
| c434093 | table-driven GPIO, commutation, block brake and LED code (-1.2 KB) |
| 41bff1f | CANSimple: negative values stored as 0 in config words (they wrapped: -2.5 V undervoltage = 63 V) |
| 932084a | mm32flash: UART DMA stopped while halted (EN = 0 does not stop it), reprogram on verify failure |
| 6746b49 | PinFinder 2.0 in this tree (TUI + agent mode, `tools/pinfinder.py`), shared commutation/config code, `tools/cfgfields.py`; passive buzzer; charger state over CANSimple |

Verification for every change: flashed to the bench board and checked with RemoteUartBus telemetry
(bus replies, speed steps, CPU load by IPSR sampling); refactors also by comparing `objdump -d -r` of
the objects before/after.
