"""Board configuration in flash (sim_eeprom.c), shared by mm32flash.py and mm32config.py.

Current layout (2026-10-30): one 1 KB page at 0x08007C00, 8 slots of 128 bytes. A save writes a pair:
the extension record (words 64-127, EXT_MAGIC at its word 32) and then the main record (words 0-63, MAGIC
at word 32) in the next slot, each record's magic written last; when fewer than two blank slots are left
the page is erased first. The newest main record is the last slot with the main magic before the first
blank one (a save cut short by a power loss leaves a slot without it), its extension the slot before it.

Old layout (until 2026-10-30): two pages at 0x08007800, 16 slots used round, the newest record the slot
before the first blank one (with wrap). read() still reads it, so flashing a new image keeps the config of
a board that ran the old layout (mm32flash.py writes it into the new page). The old scheme erased its
first page when it wrote slot 15: with only the second page in use the records sit in the new page in the
order the new rules read them, so that case is read as the new layout (and needs no move).
"""
import struct

CFG_BASE, CFG_SIZE = 0x08007C00, 0x400
OLD_BASE, OLD_SIZE = 0x08007800, 0x800
SLOT = 128
MAGIC, EXT_MAGIC = 0xDCAB, 0xDCAD    # word 32 of the main / the extension record


def _words(area, i):
    return list(struct.unpack("<64H", area[i * SLOT:(i + 1) * SLOT]))


def _blank(area, i):
    return area[i * SLOT:(i + 1) * SLOT] == b"\xff" * SLOT


def _pair(area, main, ext):
    """128 words from the main record and its extension (defaults when the extension is not valid)"""
    base = _words(area, main)
    e = _words(area, ext) if ext is not None else None
    if e is None or e[32] != EXT_MAGIC:
        e = [0xFFFF] * 64
        e[32] = EXT_MAGIC
    return base + e


def parse_new(page):
    """(main slot, 128 words) of a 1 KB page in the current layout, or (None, None)"""
    n = len(page) // SLOT
    first = next((i for i in range(n) if _blank(page, i)), n)
    # the newest slot with the main magic (the firmware writes it last: a save cut short has none)
    main = next((i for i in range(first - 1, -1, -1) if _words(page, i)[32] == MAGIC), None)
    if main is None:
        return None, None
    return main, _pair(page, main, main - 1 if main > 0 else None)


def parse_old(area):
    """(main slot, 128 words) of the old 2 KB area (16 slots, used round), or (None, None)"""
    n = len(area) // SLOT
    blank = [_blank(area, i) for i in range(n)]
    if all(blank):
        return None, None
    main = (blank.index(True) - 1) % n if True in blank else n - 1
    return main, _pair(area, main, (main - 1) % n)


def old_layout(area):
    """the old area still holds records in its first page (with the new layout that page is code)"""
    return any(struct.unpack_from("<H", area, i * SLOT + 64)[0] in (MAGIC, EXT_MAGIC) for i in range(OLD_SIZE // 2 // SLOT))


def read(t):
    """(128 words or None, 'new' / 'old' / None) from the target's flash"""
    area = bytes(t.read_memory_block8(OLD_BASE, OLD_SIZE))
    if old_layout(area):
        _, w = parse_old(area)
        return w, "old"
    _, w = parse_new(area[CFG_BASE - OLD_BASE:])
    return w, ("new" if w else None)


def write(mcu, words, fresh=False):
    """append a save (extension, then main) the way sim_eeprom.c does; fresh: erase the page first.
    mcu: mm32flash.Mm32 with the core halted. Returns the main record's slot."""
    page = bytes(mcu.t.read_memory_block8(CFG_BASE, CFG_SIZE))
    n = CFG_SIZE // SLOT
    first = next((i for i in range(n) if _blank(page, i)), n)
    if fresh or first > n - 2 or any(not _blank(page, i) for i in range(first, n)):
        mcu.erase(CFG_BASE, CFG_BASE + CFG_SIZE)
        first = 0
    for k, part in enumerate((words[64:], words[:64])):
        a = CFG_BASE + (first + k) * SLOT
        mcu.program_at(a, struct.pack("<64H", *part))
        if list(struct.unpack("<64H", bytes(mcu.t.read_memory_block8(a, SLOT)))) != list(part):
            raise SystemExit("config verify of slot %d FAILED" % (first + k))
    return first + 1
