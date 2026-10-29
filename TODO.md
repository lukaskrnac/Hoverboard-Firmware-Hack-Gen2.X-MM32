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
      byte-identical). The detections that need a person at the board (button, charger, latch, LEDs,
      `poweroff`, `save`) were tried by the user on 2026-10-28: working.
- [x] **P2 Agent mode** (`@` lines, `> cmd` echo, `key=value`, `ok`/`err`) + `tools/pinfinder.py` (`--json`),
      README "Autodetect&configuration (PinFinder)". Long commands stop on any received byte.
- [x] **Web setup wizard (2026-10-28)**: pinfinder.html Setup tab (13 steps), Advanced = the direct controls;
      E2E `uitest/wiztest.mjs` walks every step on the bench board (no save at the end). Fixed on the way:
      a UART receive error (adapter dropping off at a USB hub reset) stopped the rx DMA in both firmwares
      until cleared (uart.c clears it now); PinFinder drops line noise before an '@'; `pin blink` toggles
      the pulls instead of driving the pin. Battery step (console 419699c): battery monitoring on/off, Li-ion
      cell count fills full / empty (4.2 / 3.2 V per cell), limits checked against the firmware's
      "<= 20 V = off"; off sets `batempty` 0, `batfull` stays (regeneration limit). Tested in headless Chrome.
- [ ] **P3 TUI**: done: one screen with the pin map (unsaved changes yellow), VDD/battery, menu, LED
      screen, test motor screen, command line = agent commands, plain text without ANSI. Open: per item
      help (`?` shows the command list only), "how found" per pin (detected/manual/default), a screen
      for the remaining config words. Done 2026-10-30: detect all (`a`) runs hall, current, battery, button
      (master), charger, IMU, then a summary (ok / FAILED / skipped by a key), then the LED screen; Esc stops
      the rest (at the next step that reads keys). Bench: hall, current, IMU ok; battery FAILED (two pins near
      the bus voltage: it asks for volts), button / charger skipped.
- [x] **P4 Web GUI config tool**: `pinfinder.html` in the console repo (remoteuartbus-online 'console'
      branch): detections with progress/Stop, pin map + all words (`tools/cfgfields.json`, copied there),
      RAM edits marked until Save, Revert, test motor, pin actions, export/import JSON, command line.
      E2E in headless Chrome against the bench board (`uitest/pfuitest.mjs`: connect, edit/revert, pin
      set, hall, IMU, rotate, Stop; no page errors). Open: rows are marked against the RAM state at
      connect (an earlier unsaved PinFinder session only shows as the "unsaved" pill; PinFinder has no
      command to read the flash record); the console's main-firmware Settings could use cfgfields.json too.
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
        I_x = -(I_a + I_b): 8 linear points, 2.35 counts/A, zero -3.5..-4.6 counts -> itotaldivider ~330,
        itotalzero ~400 (blockcal.py's 419 / 145 was wrong: the shunt is R003, see Calibrations). Boards
        without phase current pins: the bus pin is not searched yet (needs a reference: motorr).
      - **Done (`detect imu`):** pull-up pins -> I2C pairs (a pair where > 8 addresses acknowledge is not I2C:
        bench PB3), id registers. Bench: unknown chip at 0x69 on SDA PD0 / SCL PD1, WHO_AM_I (0x75) = 0x2E,
        reg 0x0F = 0x7D, reg 0x00 = 0 (a clone? identify, then read it in the main firmware). Words 83-86.
        Original plan: scan I2C on the hardware I2C pin pairs (PD0/PD1 on the bench board) and bit-banged on
        other free pin pairs with pull-ups; probe 0x68/0x69 (MPU-6050/6500/9250, ICM-20xxx: WHO_AM_I
        0x75), 0x6A/0x6B (LSM6DS*: 0x0F), BMI160 (0x68, CHIP_ID 0x00 = 0xD1); report bus pins, address,
        chip. Store pins + type in new config words (extension record).
        **Main firmware (2026-10-28):** `Src/imu.c` + `Src/i2c.c` (shared with PinFinder): the bench chip is an
        MPU-6050 clone (register map read over SWD: accel offsets 0x06, self test 0x0D-0x10, WHO_AM_I
        0x2E); 100 Hz, one byte per main loop pass (a whole burst blocked ~2 ms, then ~1 ms); CANSimple
        0x150-0x159. Mounting (`calibrate imu`, words 97-100): board axes, gyro bias, pitch / roll by a
        complementary filter (bench: signs and level verified by tilting the board). PinFinder space: the
        config name table and the IMU chip table packed (-480 B), messages shortened, UART baud from
        SystemCoreClock (-150 B in both images). Open: other register maps (ICM-426xx, BMI160/270, LSM6DS*,
        QMI8658); a non axis-aligned mounting is only levelled (pitch / roll stay about the nearest axes).
      - **Done:** charger detect pin (`detect charger`, tried by the user) and the hall angle calibration
        (`calibrate hall`, see Calibrations).
      - **Done (2026-10-28): infrared foot sensors** (`detect ir`: ir1, ir2, irmode = level + pull per sensor,
        the charger's detection as a shared helper); main firmware: `hoverboard.foot_sensors` 0x115 and heartbeat
        byte 7 bits 1-2. Not tried with real sensors (the bench board has none; the firmware side was tested
        by pulling two free pins over SWD). To make room in PinFinder: the default record built in a loop
        (PINSTORAGE_WORDS_32_47), the option lists in error messages point to help, the legacy `gen` command
        removed. Open: a policy (e.g. disarm when no foot is on the pads), if wanted.
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
- [x] **Calibrations (2026-10-28):** `calibrate hall` (edges within ~2 deg of hallcal.py, repeat +-1.5 deg;
      main firmware uses `hallang1..6`, FOC drive test passes), `calibrate poles` (needs a person turning the
      wheel: not run yet), `calibrate rl` (bench 285-298 mOhm, 308-344 uH: the 460 uH default is high),
      `calibrate kv` (block 366 / 162 vs tuned 395 / 185, sine 230 vs 242), `vbatgain` (24.87 V vs PSU 24.86),
      `detect current` with the shunt value. The bus pin choice is now the steadily rising pin over 8
      levels (PA0, an op-amp input, rose more in a 2-level test). TIM1 / UART init as register writes in
      initialize.c (identical register dump, -204 B main firmware). PinFinder 30.1 / 30 KB: little room left.
      Settled 2026-10-28: the bench shunt is R003 (3 mOhm): `detect current 1000 3` -> iphasegain 524-549
      (saved: mean 535), itotaldivider 333, itotalzero 438; blockcal.py's 419 / 145 was 25 % off. Open:
      inverted halls (P9).
- [ ] **P8 More targets.** Done (2026-10-28): `make MCU=SPIN05|SPIN06|SPIN25|F031`, `make targets` builds
      main + PinFinder for all four without warnings; the SPIN05 images are byte-identical to before. One
      header set (the SPIN05's): the vendor SVDs of all four have the same register layout for every
      peripheral we use. Differences in `Inc/mcu.h`, the pin tables in `hardware.c` (SPIN25 rewritten from
      its datasheet: the old table had UART entries without a UART number, a wrong PA6 UART2 RX, ADCCOUNT 13
      for 12 entries; F031: no UART2), `system_mm32spin05.c` (SPIN25: PLL to 72 MHz, not 96) and
      `adc_Init()` (SPIN25 op-amps OP1-3 for configured current pins). Old Keil SPIN25 files removed.
      MM32F031 board: same die as the SPIN05 (IDCODE 0xCC4460B1, 32 KB), PinFinder answers on PB6/PB7;
      paused (hardware trouble on that board). Stock firmware: `backups/` (not in git).
      Open:
      - SPIN06 / SPIN25 untested on hardware; their IDCODEs are unknown. Runtime check (PinFinder
        reports the chip and refuses a firmware for another MCU; the flasher checks before programming):
        needs the IDCODEs first. SPIN05 and F031 cannot be told apart by IDCODE (same die).
      - PinFinder on SPIN25: `detect current` scans ADC pins, but the op-amp outputs only drive their pins
        when enabled; enable OP1-3 during the scan (their pins must be free).
      - SPIN06 has 16 KB RAM / 64 KB flash, SPIN25 8 KB RAM: unused (all images use the 32 KB / 4 KB layout).
      - MM32F030 / the `origin/pinfinder-spin280` branch's MM32SPIN0280: not looked at.
- [ ] **P9 Still not detected / features to consider** (review, then promote to items):
      - E-stop input (bench PB0) and WS2812 header (PB12): detect + support (E-stop = disarm like the
        CANSimple Estop; WS2812 status LED like the RGB LED).
      - (Done since: pole count, motor R / L, kv, battery divider, battery cell count -> full / empty: see
        Calibrations and the wizard's Battery step.)
      - Low-side polarity (`INVERT_LOWSIDE`) and the over-current comparator pins (`OCPPIN`/`OCPREFPIN`):
        detect safely (gate driver input states with the bridge unpowered?) or keep manual with a warning.
      - Master/slave UART (the second board link) and the remote UART pins: `finduartloop` exists for
        the debug port; extend to both.
      - Temperature: MCU internal sensor (ADC channel) and an NTC pin if fitted; report + derate.
      - Headlight / brake light outputs on some boards (detect like the LEDs).
      - Hall inputs inverted / 180 deg off the commutation: PinFinder detects it (no assignment fits, the
        inverted one does) but the firmware cannot use it; a `hallinvert` config bit (hallpos() 7-state)
        would make such motors work.
      - Hall edge positions at 30 deg resolution come out of `detect hall` already (12 directions); the
        hall angle calibration (P5) can refine them.

## Next

- [ ] Bus current reading (2026-10-30): in sine above SAMPLE_MAX (the phase samples invalid, ~10 turns/s up
      with field advance) itotal froze at its last phase value (user: "it doesn't read current any more");
      it now stays the shunt's active-vector reading (busv runs in sine always). That reads 1.5-2.2x the
      supply there (333 / 495: 1.09 / 2.48 / 4.75 A vs 0.70 / 1.32 / 2.18 A at 11 / 13 / 16 turns/s).
      A divider of 120 matched the supply at those speeds but was WRONG (reverted the same day): the shunt
      per sample follows 2.2-2.35 counts/A against the phase currents both static (PinFinder) and running
      (logged at FOC 8.3 / sine 8 turns/s: binned means, 2-3 % spikes), so 333 / 495 is right and 120 made
      block mode, softocp and the watchdog read 2.8x low. The cause of the high reading: the adc clips at
      0, so the shunt cannot see negative (regenerating) window currents, which field advance makes large;
      the sum of the positive parts is above the net bus current. Open: a better estimate above SAMPLE_MAX
      (power balance from the commanded voltages, or the window sign from the current angle); until then
      the shunt reading is an upper bound (conservative for softocp).
- [ ] (old note) Bus current in sine mode on boards **without** phase current pins: done 2026-10-28 (interrupt.c): after
      each control step the adc trigger moves to TIM1 CC4 in the middle of one of the two active vector windows
      (alternating), a short second isr run reads the shunt there and sets the trigger back to TRGO; bus current
      = (window 1 x I1 + window 2 x I2) / period; windows < 6 us count 0; SysTick restores the trigger if a
      sample never comes. Bench without the phase pins (config), sine 2 turns/s, braked by hand: follows the
      load (0.1-1.2 A) but reads 1.5x the supply's current at every load. Block, braked, same board: 1.6-1.85x.
      So the bus shunt calibration from PinFinder's detect current (itotaldivider 333 / itotalzero 495, derived
      from the phase currents) reads ~1.8x high on the bench board; not recalibrated against the supply (that
      would only fix this board). If detect current is revisited: check its bus divider against a known load.
- [x] Position, sine (2026-10-30): was P on whole hall steps (2 rpm per step, +-1 step dead band, no stopping
      limit): half a turn 1.7-1.9 s, stopped a step short, 10 turns at vel_limit 30 asked 1800 rpm: 1.2 turns
      overshoot, once 5.5 and CURRENT_LIMIT_VIOLATION. Now the FOC path's fine P, sqrt(2 a err) stopping limit
      and accel slew, half step dead band; sine gain 60. Sine: 0.5 / 3 / 10 turns in 0.47 / 1.6 / 2.6 s settled,
      overshoot 0.005 / 0.06 / 0.06, exact (FOC 0.44 / 1.2 / 2.25 s, 0). Block uses the same path (gain 20):
      exact now, still overshoots (0.4 turns at 3, 1.5 at 10; was 0.4 / 1.73 and a step short): it cannot brake.
- Block position overshoot: inherent (block cannot brake while moving; user 2026-10-30: always like this, not
  pursued). A 10 turn move once pumped the bench supply into its 41 V OVP with the old firmware.
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
      2026-10-28: partial field weakening fixed (the speed loop on realspeed with field weakening too: 5 A
      0.5-0.7 m/s^2, was 1.7-2.0). With 9 A both speed sources shake alike: 2.8 m/s^2 and +-14 rpm at 680 rpm
      (I_d -7.5..-8 A), 1.6 at 740: the large d current itself (this item), not the speed measurement.
      2026-10-28 (phone accelerometer, free wheel, 24.5 V): the shaking needs a large d current, not field
      weakening or speed: I_d forced (id_setpoint 0x112) at 300 rpm, no field weakening: 0 A 0.54 m/s^2, -3 A
      0.8, -4.5 A 4.6, -5.5 A 13.8, -6 A 6-21 m/s^2 (growing; speed +-100 rpm, a ~52 Hz oscillation). Ruled out:
      the speed source (realspeed / fastspeed), the field weakening integrator (P terms out of its input,
      low pass, 4x slower: no change), the hall tracker gain (1/4..1/32 per edge at -4.5 A: 4.2-4.7), a frame
      shift proportional to I_d (k LSB per A along the motion: at 680 rpm / 9 A field weakening -1000..-1300
      halved the vibration and needed less I_d, but -1600 lost torque and tripped CURRENT_LIMIT_VIOLATION; at
      300 rpm with forced I_d it made it worse). FOC trace (TRACE=1, every 4th period, captured by a UART
      trigger, read over SWD stopped) at -4.5 A / 300 rpm: the tracker frame is smooth (+-0.8 deg el off a
      line), the dq currents small (+-0.16 / 0.06 A), while the rotor swings ~+-7 deg el (+-25 rpm at ~50
      Hz): the frame does not follow the rotor in that band and I_d sin(delta) with I_d < 0 is a negative
      spring. Fix = a rotor angle with bandwidth above ~50 Hz: a back emf / flux observer fused with the
      halls (sizeable). Until then fwimax <= ~5 A (README). Do not force I_d below -5 A again (2 g of
      shaking); never SWD while it runs (wedges the ST-Link).
      Speed dependent (36 V, I_d forced, no field weakening): -4.5 A shook at 300 rpm only (450-750 calm), -6 A
      at 600 not at 750, 0 A nowhere: not a mechanical resonance. Field weakening now capped at 8 mA per rpm
      (smooth at any fwimax, ~647 rpm at 24 V instead of a shaking 680-733 with 9 A). More speed at the same
      voltage still needs a rotor angle that follows the wobble (observer).
      2026-10-29, rigid mount: forced I_d no longer shakes the frame (300 rpm -4.5 A 0.49 m/s^2 = the 0 A level,
      was 3.1), but the rotor still wobbles: speed sd 1.2 -> 9.7 rpm, Iq sd 0.02 -> 0.31 A (old mount: 11.6 rpm);
      520 rpm: -4.5 A calm (8.7 mA/rpm), -6 A sd 9.8 rpm (11.5 mA/rpm). The boundary is the same as on the loose
      mount (it only made it loud): the 8 mA/rpm cap stays. Field weakening with it: 0.31-0.48 m/s^2 steady
      (was 0.6-0.9), transitions 0.46-0.75, ~646 rpm at 24 V.
      Knocks with field weakening (2026-10-29, 24 V, pwmres 8192, fwimax 9 A): 12-25 short spikes (2-7 m/s^2) per
      15 s at 10-10.8 turns/s; none at 9 turns/s; none at 36 V without field weakening at the same speeds; none
      at 8192 below base speed (pwmres 2048: some at 5-6 turns/s). Not: the pullback (0), tracker snaps at speed
      (0), the angle going unsure (0), reversals at speed (0), the sample window (modMax from 3.4 to 6 us: no
      change), a tighter glitch threshold while the references stand still (no change). Seen in the trace
      (frozen by the phone's spike / in-firmware triggers): single-period d/q readings 1.5-2.5 A off (often
      +q -d), taken by the 3 A glitch filter; the loop answers with ~1 V for a period; the angles are clean.
      Experiment code: scratch fw/knock_experiments.patch, fw/knockcatch.py, fw/trknock.py.
      FOUND and fixed: the internal 1.2 V reference reading (vrefint, ADDR15, unmasked, unfiltered) now and then
      read ~26 instead of ~1489 counts: the bus voltage read 57x, MOD_MAX*v overflowed, vmax negative, one
      period with no q voltage, a 4 A current kick (trace frozen on |vq| < 2 V). The small phase sample outliers
      (~1 %, the same with and without field weakening) were bystanders. Now vrefint masked + filtered (1/16
      gate), vbat held up to 3 periods if 1/12 off, v clamped to 70 V for vmax. Bench: 0 knocks at 9-10.8
      turns/s with field weakening (was 12-25 per 15 s), 0 at pwmres 2048 / 5-6 turns/s (was 3 per 30 s).
      Room for it: the 512 byte sine table replaced by 65 entries + linear interpolation (error 3.6 instead of
      201 / 32768; low speed FOC 0.29-0.36 m/s^2, sine 0.05-0.20, was 0.39-0.59 / 0.19-0.49).
      STILL knocking at pwmres 2048 above base speed (user, web UI at max; 2026-10-29): 13-23 per 15 s at 8 turns/s
      (none at 8192). FOUND: the adc watchdog on the bus shunt (hardocp 300 counts = ~80 A at 3 mOhm) fired on
      every knock (12 hits = 12 knocks, 0 at 7 turns/s), the break cleared MOE, the loop restarted with zero
      integrators against ~10 V of back-EMF: a 7 A kick (trace: two periods missing, looked like a 20 deg angle
      jump). The shunt is sampled in the zero vector, where it carries no current; at 17.6 kHz and ~76 %
      modulation the window is a few us and the shunt (3rd in the scan) lands on a switching edge. Now the
      watchdog only breaks in block mode and sine without phase pins; sine/FOC with phase pins rely on the phase
      current trip. Bench: 0 knocks at 6/7/8 turns/s and 40 s at the top (8.35), peak 1.1-1.5 instead of 6.4 m/s^2.
      Hardware limit back (2026-10-29): sine/FOC with phase pins take the CC4 sample inside an active vector too
      (as sine without phase pins), the watchdog breaks only on that conversion. Two timing fixes: the trigger
      leads by the shunt's place in the scan (120 timer counts per channel before it + 23; bench 263: PA2, PA6
      first; it fires on the down count, measured), and windows below 9 us are skipped (at 6 us: 3 hits per
      20 s). Bench, pwmres 2048: watchdog hits at 8 turns/s 20 -> 3 -> 0 (30 s at 6 / 8 / 8.3, no trips); with
      hardocp 8 a hard step trips it on real current (12-23 counts), a gentle ramp does not. Sine without phase
      pins: windows < 9 us now count 0 in the bus current (was 6).
      2026-10-30, sine with field advance at 10 turns/s still knocked (user): 8 watchdog breaks per 30 s. The
      compare registers have no preload, so the values the control step writes act at once, also in the period
      of the CC4 sample, while its window came from the values read at the isr's start (~12 deg el earlier at
      10 turns/s). Now from the new values: 0 hits at sine 10 / 10.5 and FOC 8.3 turns/s (30 s each).
      Sine top speed with field advance is ~16 turns/s at 24 V; no watchdog hits at 11-18 turns/s commanded.
- [x] Sine drawing a lot of current (confirmed fixed by the user 2026-10-29; seen on the bench supply by the user, conditions unknown). Not
      reproduced on the unloaded wheel: at its top speed (95 % modulation) sine takes 0.4 A at 24 V
      (FOC 0.3 A), accelerations peak at ~3.8 A phase current. The board's readings froze above 90 %
      modulation until now (2.5 A shown); fixed, they are valid to 95.3 %.
      Probably the pwm period: the user saw ~100 W in sine at low speed after changing the pwm resolution.
      With the control every period the isr overran its period from ~8.8 kHz and the sine angle stood still
      (fixed 2026-10-28: the control at ~4.39 kHz whatever the pwm). Close when the user confirms.
- [ ] At high current and speed (field weakening) the ST-Link and the FTDI drop off USB now and then
      (ground noise through the shared USB ground, the hub resets). Test scripts read over the bus
      there; an isolator on the SWD/UART link would help.
- [ ] Push `gcc-build` and the web console (`remoteuartbus-online`, branch `console`; pushing to
      `main` deploys GitLab Pages). Try the console once in Firefox (WebSerial add-on, over http).

- [x] Position: during a hard reversal at speed the hall angle waits for an edge that comes late; with
      speed gains of 60/200 that once ran the currents away (30 A trip). 100/300 (position default) and
      30/100 passed every run; the tracker should handle a reversal between edges better.
      2026-10-28: no trip in 40 reversals from 230-290 rpm (target flipped mid-move, gains 30/100 .. 100/333), but
      they shake (phone accelerometer): 1.8-2.5 m/s^2 rms (peaks 10-17, 35-90 Hz) at the defaults, 6-7 at 60/200,
      9 at 100/333; holding 0.04-0.1, cruise <= 1. Only below ~100 rpm: the start, the reversal, the end of the
      braking. The supply is not involved (bus <= 26 V, below batfull). Trace (TRACE=1, 0.9 ms): iqRef swings
      -4..-6.5 A -> +2.4..+4.4 A at single hall edges where fastspeed jumps 20-25 -> 84-151 rpm within one sector,
      which 4 A cannot do (~1500 rpm/s): the low speed measurement is inconsistent while the q current swings,
      probably the edges moving with the q current (armature reaction, like the d current item above).
      Fixed meanwhile: the tracker started only after ~6 edges (sector centres, 60 deg steps; bc6789f).
      Not it: speed extrapolation between edges (tried). (A sweep of the 3.3x low speed gain was invalid:
      cfgOrDefault() takes the value, not the word index - the experiment read the constant 106.)
      Half the speed gains (15/50): 0.8 m/s^2 but 0.1 turn overshoot.
      Edge shift with the q current (q square wave +-1..2 A, 25 Hz, on FOC velocity 1-2 turns/s; tracker error
      at each edge): ~+10 deg el per A on the first edge after a flip, both directions; correcting the tracker's
      boundary by 5 deg el per A of q current (sign: bnd += 5 deg * iq in the electrical frame) removed it
      (+-13 -> +-2 deg, sd 14 -> 5.5) but the reversals only went 2.2 -> 2.0 m/s^2; correcting the intervals
      too made it worse (2.5, bus sagged to 19 V at 60/200). The error rings (later edges of a half period
      the other sign: +9 then -17 deg at 1 turn/s): a mechanical response, not instantaneous armature
      reaction. Suspect the bench mount: the stator (halls) rocking on its axle clamp under torque (1 deg
      mechanical = 15 deg el), resonant at ~40-60 Hz like the shaking. Experiment code: scratch reversal/
      foc.c.edgeshift, hallhandle.c.edgeshift (words 106 amplitude, 107 c). Check the mount first.
      2026-10-29, the motor in a rigid mount (printed v4, ~/Documents/3ddesign/motormount): the same reversals
      0.62-0.75 m/s^2 rms (peaks 3.5-6, 120-180 Hz) instead of 1.8-2.5 (peaks 10-17, 35-70 Hz), no trips, no
      overshoot, also at vel_limit 30 turns/s in PASSTHROUGH and POS_FILTER: most of it was the loose mount.
      2026-10-30, rigid mount, current firmware: the edge shift with the q current is gone (q square wave +-1 / +-2 A at
      1 and 2 turns/s: first edge after a flip -0.6..+0.3 deg el, all edges within +-0.7, sd 3-5; loose mount: ~+10 deg
      el per A): it was the stator rocking in the loose clamp, no tracker correction needed. Reversals from 4.8 turns/s
      (PASSTHROUGH, vel_limit 30): default gains 30/100 0.70-0.82 m/s^2 rms (peaks 3-8), no overshoot, exact arrival,
      no errors in 10; 100/300 2.3-3.1 rms (peaks 14-30), also no errors: too stiff for this wheel, keep the default.
      Experiment hooks (q square wave word 120, edge log) in scratch reversal/edgeshift2.py, not in the tree.
      Steady speed (rigid mount, phone on the mount): 0.35 / 0.69 / 0.55 m/s^2 at 4 / 6 / 8 turns/s, peaks at
      29.6x and 26.6x the wheel frequency (2x electrical = 30 magnets, and the 27 stator slots; the 1.3 % below
      30 / 27 is probably the HSI clock). The same in sine (no current loop): 0.37 / 0.62 / 0.53; unchanged by
      a channel 2 gain trim of +-4 %, curkp 200-800, focspdkp 15-60. Motor + structure (resonance ~160-180 Hz),
      not the control. The short bumps at speed were the current glitch filter (e58e209).
      Audible noise (phone mic, UNPROCESSED, dB added by the running wheel over standing, 4 / 7 turns/s):
      pwmres 8192 (4.4 kHz) +15.0 / +17.8, 6-12 kHz +33 (whine at 4.4 / 8.8 / 13 kHz); 4096 +8.7 / +16.8;
      2048 (17.6 kHz) +7.4 / +13.2, 6-12 kHz +4 / +12, the rest at 17.6 kHz. Below 1 kHz all alike (the motor).
      2048 costs the modulation limit (76 % instead of 94 %). The bench board runs 2048 (user's choice, 2026-10-29:
      top 7.25 turns/s = 435 rpm at 24 V without field weakening); the firmware default stays 8192.
- [ ] Flash. PinFinder (2026-10-29): messages, help and config names byte pair compressed at build time
      (tools/pfstrings.py, 128 pairs, decoded in out() / eq()): SPIN05 30624 -> 28964 B (1.6 KB free), SPIN25
      28996, F031 28932; agent and TUI output byte-identical on the bench board. Next room there: the 0x01-0x1F
      bytes as more pair codes, Command()'s if-chain as a table, the soft float in the calibrations.
      Main firmware SPIN05 30596, SPIN25 30700 (20 B free; the boot "hello World" on the bus line removed,
      adc_Init()'s three adc table searches merged).
      Earlier (2026-10-29, hardware I2C/buzzer + PinFinder pin checks): main SPIN05 30564 of 30720 B, SPIN25 30704 (16 B free), F031 30540; PinFinder SPIN05 30632, SPIN25 30660, F031 30608 (PinFinder messages shortened for it). After the sine table: main SPIN05 30276, SPIN25 30392; PinFinder SPIN05 30688, SPIN25 30720. Was: 29.1 of 30 KB (1.6 KB free; PinFinder 30.6 of 30 KB, 104 B free, SPIN25 32 B), RAM 1.4 KB + 1 KB stack reserve of 4 KB (worst case stack
      ~670 bytes, GCC's call graph `-fcallgraph-info=su` on the LTO link). Next room if needed: the
      vendor HAL init calls inlined into main (TIM1 / ADC / UART init through structs), `odriveParam()`
      (1.2 KB, a switch per endpoint), the 512 byte sine table.
- [ ] ODrive settings not implemented: encoder bandwidth (the hall tracker's gains are per edge and
      measured, see hallhandle.c), calibration_current / motor calibration, dc_max_positive/negative_current
      (bus current limits; the regeneration limit is by voltage, `batfull`), spinout detection.
- [x] Bus line above 115200 baud (2026-10-30): config `txpushpull` (word 94, 2 = on) drives TX push-pull for
      one board alone on the line. Bench (FT232R): open drain 115200 500/500, 230400 7/500, 460800 0/500;
      push-pull 460800 and 921600 500/500. Open: the web console's copy of cfgfields.json (console repo).

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
- [ ] Host unit tests for the pure logic (CRC and frame layout against the web console's test
      vectors, SpeedCtrl() against the measured plant) and a CI build. Needs the logic split from
      the HAL-dependent code first.
- [ ] Naming is mixed (camelCase, snake_case, Hungarian `iSpeed`/`bConfigRx`, vendor `u8`/`s32`
      next to stdint; config macros like `DRIVEMODE` are lvalues of `pinstorage[]`): clean up file by
      file when a file gets real work, not in one sweep.
- [ ] RemoteUartBus: state bit 128 (shut off) and `slaveNew` are not implemented; there is no read
      command for the configuration (the console cannot show the board's drive mode). A position mode
      (GD32 drive mode 3, odometer target) would be the useful new mode.
- [ ] Peripheral paths not tested on hardware yet: an IMU that does not answer (TX_ABRT -> state 2),
      the passive buzzer's sound (TIM17 CH1 on PB9: registers verified, 2.2 kHz / 50 %, not listened
      to), TIM14/TIM16 and the CH1N pins, a hall wiring that needs a pin's second channel (PA0 AF2 / PB3 AF2/AF4: host-tested only),
      UART2 with the I2C DMA, SPIN25 / F031 at all (tables from their datasheets).
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
