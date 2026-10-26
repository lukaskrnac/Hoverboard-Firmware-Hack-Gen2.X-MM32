# MM32SPIN25 (not ported to GCC yet)

`system_MM32SPIN2xx_p.c` and `startup_MM32SPIN2xx_p_keil.s` are the Keil RTE files of the old
`MM32SPIN25` Keil target. They are kept for the GCC port, which still needs:

- the MindMotion MM32SPIN2x device headers (`HAL_device.h`, `MM32SPIN2xx_p.h`), which came from
  the Keil pack and were never in this repository,
- a GCC startup file and linker script (see `../../MM32SPIN0x/Source/` for the SPIN05 ones),
- a Makefile target with `-DTARGET_MM32SPIN25` and `Library/MM32SPIN25/HAL_lib`.
