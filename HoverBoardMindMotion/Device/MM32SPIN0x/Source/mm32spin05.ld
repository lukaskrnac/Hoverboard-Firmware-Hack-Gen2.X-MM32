/* MM32SPIN05: 32 KB flash, 4 KB RAM.
   The last 2 KB of flash (0x08007800) hold the sim-EEPROM config (sim_eeprom.c) and are kept
   out of the image so a firmware update never overwrites the board configuration. */
MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 30K
    CONFIG (r)   : ORIGIN = 0x08007800, LENGTH = 2K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x400;

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH

    .ARM.exidx : { *(.ARM.exidx*) } > FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        end = .;
    } > RAM

    /* fails the link if less than _min_stack bytes are left for the stack */
    .stack_check (NOLOAD) : { . = . + _min_stack; } > RAM
}
