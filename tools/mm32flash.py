#!/usr/bin/env python3
"""Flash MM32SPIN05/06 firmware over SWD with pyOCD (any probe pyOCD supports; no vendor tools).

    python3 tools/mm32flash.py HoverBoardMindMotion/build/hoverboard.bin
    python3 tools/mm32flash.py image.bin --backup before.bin       # dump all 32 KB first
    python3 tools/mm32flash.py image.bin --erase-config            # also wipe the board config
    python3 tools/mm32flash.py image.bin --start reset             # system reset instead of soft start
    python3 tools/mm32flash.py board-id3.bin                       # an image with its config (image.html)

pyOCD has no MM32 flash algorithm, so this erases pages through the FLASH registers and programs
with a 26-byte RAM routine (the CPU does the halfword stores; ST-Link 16-bit debug writes program
every odd halfword as 0 on this chip).

The last 1 KB (0x08007C00) holds the board configuration written by PinFinder / CANSimple / the
RemoteUartBus config command (mm32cfgstore.py). It is kept unless --erase-config is given; a board with
the old layout (2 KB at 0x08007800, until 2026-10-30) gets its configuration moved into the new page.
An image longer than the code area (the web console's image.html: firmware + config page) is written as a
whole, its config replacing the board's.

--start soft (default) starts the new firmware without a system reset: a reset floats every GPIO,
which releases the power self-hold latch and switches the board off. Instead the APB peripherals
are reset (GPIO is on AHB and keeps the latch), interrupts are silenced, any active exception is
left through an exception return, and the core jumps to the reset vector.
"""
import argparse
import os
import struct
import time

from pyocd.core.helpers import ConnectHelper

FLASH_REG = 0x40022000
FLASH_BASE, FLASH_SIZE, PAGE = 0x08000000, 0x8000, 0x400
from mm32cfgstore import CFG_BASE as CONFIG    # noqa: E402  (the 1 KB config page)
import mm32cfgstore                            # noqa: E402
RCC = 0x40021000
IWDG_KR, DBGMCU_CR = 0x40003000, 0x40013404
RCC_APB2ENR_DBGMCU, DBGMCU_CR_IWDG_STOP = 1 << 22, 1 << 8
DMA1_CCR = [0x40020008 + 0x14 * n for n in range(5)]      # channel 1..5 configuration (bit 0 = EN)
UART_GCR = [0x40013818, 0x40004418]                         # UART1, UART2 global control (bit 1 = DMA requests)
CORE_REGS = ["r%d" % i for i in range(13)] + ["sp", "lr", "pc", "xpsr", "msp", "psp", "primask", "control"]

# r0 = dst, r1 = src, r2 = halfword count, r3 = FLASH_REG:
#   loop: ldrh r4,[r1]; strh r4,[r0]; wait: ldr r5,[r3,#12]; movs r6,#1; tst r5,r6; bne wait
#         adds r0,#2; adds r1,#2; subs r2,#1; bne loop; bkpt
PROGRAM_STUB = struct.pack("<13H", 0x880C, 0x8004, 0x68DD, 0x2601, 0x4235, 0xD1FB,
                           0x3002, 0x3102, 0x3A01, 0xD1F5, 0xBE00, 0xBF00, 0xBF00)
STUB_AT, BUF_AT, CHUNK, STACK_AT = 0x20000100, 0x20000400, 0x800, 0x20000FF0
RAM, RAM_SIZE = 0x20000000, 0x1000                          # the routines use the firmware's RAM
TIM1_BDTR, BDTR_MOE, BDTR_AOE = 0x40012C44, 1 << 15, 1 << 14   # the bridge outputs: main / automatic output enable


class Mm32:
    def __init__(self, target):
        self.t = target

    def r(self, a):
        return self.t.read32(a)

    def w(self, a, v):
        self.t.write32(a, v)

    def halt(self):
        """Halt the core with the independent watchdog frozen and the DMA stopped.
        The firmware runs the watchdog at ~500 ms; it cannot be stopped once started and its reset
        powers latched boards off. On this MM32 DBGMCU IWDG_STOP freezes it whenever debug is
        enabled, not only while halted, so soft_start() clears the bit again.
        A halt does not stop the firmware's UART RX DMA, which writes received bus bytes into RAM
        (its ring can overlap the programming buffer at BUF_AT: a frame from a polling master landed in
        the flash image). Clearing the channel's EN bit does not stop it on this MM32 (measured: 60
        bytes arrived with EN = 0), so the UARTs' DMA requests are switched off as well.
        TIM1 keeps running while the core is halted: the bridge went on switching with the last compare
        values, a static voltage vector (against the back emf at speed). Its main output enable (and the
        automatic one, which would set it again at the next update) is cleared first; resume() sets them
        back. DBGMCU's TIMx_STOP only freezes the counter, which can leave a phase switched on."""
        self.t.halt()
        self.saved_bdtr = self.r(TIM1_BDTR)
        self.w(TIM1_BDTR, self.saved_bdtr & ~(BDTR_MOE | BDTR_AOE))
        self.saved_regs = [(r, self.t.read_core_register(r)) for r in CORE_REGS]
        self.saved_uart = [(gcr, self.r(gcr)) for gcr in UART_GCR]
        for gcr, v in self.saved_uart:
            self.w(gcr, v & ~2)
        self.saved_ram = self.t.read_memory_block32(RAM, RAM_SIZE // 4)
        self.saved_dma = [(ccr, self.r(ccr)) for ccr in DMA1_CCR]
        for ccr, v in self.saved_dma:
            self.w(ccr, v & ~1)
        self.w(RCC + 0x18, self.r(RCC + 0x18) | RCC_APB2ENR_DBGMCU)
        self.w(DBGMCU_CR, self.r(DBGMCU_CR) | DBGMCU_CR_IWDG_STOP)
        self.feed()

    def resume(self):
        """Continue the firmware where halt() stopped it. The RAM routines overwrite core registers
        (a plain resume ran into their bkpt: HardFault) and the firmware's RAM (resumed without the
        restore, it ran with garbage in its filters: 60 A telemetry), so both are put back, then the
        DMA and the watchdog."""
        self.t.write_memory_block32(RAM, self.saved_ram)
        for reg, val in self.saved_regs:
            self.t.write_core_register(reg, val)
        for ccr, v in self.saved_dma:
            self.w(ccr, v)
        for gcr, v in self.saved_uart:
            self.w(gcr, v)
            if v & 2:
                # the bytes received while halted overran the receiver, and a byte already waiting
                # makes no new DMA request: the firmware received nothing until the data register
                # was read (measured). Drop it and clear the overrun flag.
                self.r(gcr - 0x18 + 0x04)                   # RDR
                self.w(gcr - 0x18 + 0x14, (1 << 1) | (1 << 3))    # ICR: RX, RXOERR
        self.w(DBGMCU_CR, self.r(DBGMCU_CR) & ~DBGMCU_CR_IWDG_STOP)
        self.w(TIM1_BDTR, self.r(TIM1_BDTR) | (self.saved_bdtr & (BDTR_MOE | BDTR_AOE)))    # the gates as they were
        self.feed()
        self.t.resume()

    def feed(self):
        self.w(IWDG_KR, 0xAAAA)                             # reload; harmless if it is not running

    def unlock(self):
        if self.r(FLASH_REG + 0x10) & 0x80:
            self.w(FLASH_REG + 0x04, 0x45670123)
            self.w(FLASH_REG + 0x04, 0xCDEF89AB)
        if self.r(FLASH_REG + 0x10) & 0x80:
            raise RuntimeError("flash is still locked")
        self.w(FLASH_REG + 0x0C, 0x34)                      # clear EOP, WRPRTERR, PGERR

    def wait(self):
        while self.r(FLASH_REG + 0x0C) & 1:
            pass
        sr = self.r(FLASH_REG + 0x0C)
        if sr & 0x14:
            raise RuntimeError("flash error, SR=%08x" % sr)

    def erase(self, start, end):
        self.unlock()
        for a in range(start, end, PAGE):
            self.w(FLASH_REG + 0x10, 0x2)                   # PER
            self.w(FLASH_REG + 0x14, a)
            self.w(FLASH_REG + 0x10, 0x42)                  # PER | STRT
            self.wait()
        self.w(FLASH_REG + 0x10, 0x80)                      # LOCK

    def run(self, regs, timeout=3.0):
        for reg, val in regs:
            self.t.write_core_register(reg, val)
        self.feed()
        self.t.resume()
        t0 = time.time()
        while self.t.get_state().name != "HALTED":
            if time.time() - t0 > timeout:
                self.t.halt()
                raise RuntimeError("RAM routine did not finish")

    def program_at(self, addr, data):
        """Program erased flash at addr (halfword aligned, len(data) even and <= CHUNK); core halted."""
        self.t.write_memory_block8(STUB_AT, list(PROGRAM_STUB))
        self.t.write_memory_block8(BUF_AT, list(data))
        self.unlock()
        self.w(FLASH_REG + 0x10, 0x1)                       # PG
        self.run((("r0", addr), ("r1", BUF_AT), ("r2", len(data) // 2),
                  ("r3", FLASH_REG), ("sp", STACK_AT), ("pc", STUB_AT),
                  ("xpsr", 0x01000000), ("primask", 1)))
        if self.t.read_core_register("r2") != 0:
            raise RuntimeError("programming stopped at 0x%08x" % addr)
        self.w(FLASH_REG + 0x10, 0x80)                      # LOCK

    def program(self, image):
        for off in range(0, len(image), CHUNK):
            chunk = image[off:off + CHUNK]
            if chunk != b"\xff" * len(chunk):
                self.program_at(FLASH_BASE + off, chunk)

    def soft_start(self):
        for off in (0x0C, 0x10):                            # APB2RSTR, APB1RSTR
            self.w(RCC + off, 0xFFFFFFFF)
            self.w(RCC + off, 0)
        self.w(0xE000E010, 0)                               # SysTick off
        self.w(0xE000E180, 0xFFFFFFFF)                      # NVIC ICER
        self.w(0xE000E280, 0xFFFFFFFF)                      # NVIC ICPR
        self.w(0xE000ED04, (1 << 25) | (1 << 27))           # ICSR: clear pending SysTick / PendSV
        # writing PC does not leave handler mode: return from each active exception through a
        # fake frame whose return address is a bkpt
        ret_at, frame_at = 0x20000E00, 0x20000F00
        self.t.write_memory_block8(ret_at, [0x70, 0x47, 0x00, 0xBE])      # bx lr ; bkpt
        for _ in range(8):
            if self.t.read_core_register("xpsr") & 0x3F == 0:
                break
            self.t.write_memory_block8(frame_at, list(struct.pack("<8I", 0, 0, 0, 0, 0, 0, ret_at + 2, 0x01000000)))
            self.run((("msp", frame_at), ("sp", frame_at), ("lr", 0xFFFFFFF9), ("pc", ret_at), ("primask", 0)), 1.0)
        if self.t.read_core_register("xpsr") & 0x3F:
            raise RuntimeError("could not leave handler mode")
        sp, pc = self.r(FLASH_BASE), self.r(FLASH_BASE + 4)
        for reg, val in (("msp", sp), ("sp", sp), ("pc", pc & ~1), ("xpsr", 0x01000000),
                         ("primask", 0), ("control", 0)):
            self.t.write_core_register(reg, val)
        self.w(DBGMCU_CR, self.r(DBGMCU_CR) & ~DBGMCU_CR_IWDG_STOP)    # watchdog runs again
        self.feed()
        self.t.resume()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image", help="raw binary for 0x08000000")
    ap.add_argument("--erase-config", action="store_true", help="also erase the board configuration (1 KB)")
    ap.add_argument("--backup", metavar="FILE", help="save the whole 32 KB flash before writing")
    ap.add_argument("--start", choices=("soft", "reset", "none"), default="soft",
                    help="how to start the new firmware (default: soft, keeps the board powered)")
    ap.add_argument("--frequency", type=int, default=1000000, help="SWD clock in Hz")
    ap.add_argument("--probe", default=os.environ.get("MM32_PROBE"), metavar="UID",
                    help="probe unique id when more than one is connected (or MM32_PROBE); pyocd list shows them")
    a = ap.parse_args()

    image = open(a.image, "rb").read()
    image += b"\xff" * (len(image) & 1)
    with_cfg = FLASH_BASE + len(image) > CONFIG    # firmware + config (image.html)
    if FLASH_BASE + len(image) > FLASH_BASE + FLASH_SIZE:
        raise SystemExit("image is %d bytes, more than the %d of flash" % (len(image), FLASH_SIZE))
    if with_cfg and mm32cfgstore.parse_new(image[CONFIG - FLASH_BASE:])[1] is None:
        raise SystemExit("image is %d bytes and reaches into the config page at 0x%08x without a valid config record"
                         % (len(image), CONFIG))

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=a.probe, target_override="cortex_m", frequency=a.frequency,
        options={"connect_mode": "attach", "resume_on_disconnect": True})
    session.open()
    mcu = Mm32(session.target)
    try:
        mcu.halt()
        if a.backup:
            open(a.backup, "wb").write(bytes(mcu.t.read_memory_block8(FLASH_BASE, FLASH_SIZE)))
            print("backup:", a.backup)
        cfg, layout = (None, None) if a.erase_config or with_cfg else mm32cfgstore.read(mcu.t)
        # always erase the whole application area, so no code of an older, bigger image survives
        end = FLASH_BASE + FLASH_SIZE if a.erase_config or with_cfg else CONFIG
        for attempt in range(3):
            mcu.erase(FLASH_BASE, end)
            mcu.program(image)
            back = bytes(mcu.t.read_memory_block8(FLASH_BASE, len(image)))
            bad = [i for i in range(len(image)) if back[i] != image[i]]
            if not bad:
                break
            print("verify failed at +0x%x (%d bytes differ), programming again" % (bad[0], len(bad)))
        else:
            # resuming would run the old firmware's code from wherever it stopped in the new, corrupt image
            session.options.set("resume_on_disconnect", False)
            raise SystemExit("verify FAILED at +0x%x (%d bytes differ); core left halted with the watchdog frozen "
                             "(the board stays on): flash again" % (bad[0], len(bad)))
        print("programmed and verified %d bytes, erased 0x%08x-0x%08x%s" %
              (len(image), FLASH_BASE, end, " (config from the image)" if with_cfg else " (config erased)" if a.erase_config else ""))
        if layout == "old":    # its first page was code space now (erased above): into the new page
            mm32cfgstore.write(mcu, cfg, fresh=True)
            print("config moved from the old 2 KB layout to the 1 KB page at 0x%08x" % CONFIG)
        if a.start == "soft":
            mcu.soft_start()
            print("started (soft start, board stays powered)")
        elif a.start == "none":
            session.options.set("resume_on_disconnect", False)
            print("core left halted (watchdog frozen); start it with a soft start or a power cycle")
        elif a.start == "reset":
            print("reset: the board powers off if it is held on by its self-hold latch")
            try:
                mcu.t.reset()
            except Exception:
                pass
    finally:
        try:
            session.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
