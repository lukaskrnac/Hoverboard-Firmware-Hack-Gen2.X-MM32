#!/usr/bin/env python3
"""PinFinder message compression (build step, see the Makefile).

PinFinder's messages are ~6 KB of its 30 KB. This copies PinFinder/*.c to an output directory with the
printed string literals byte-pair encoded: bytes 0x80-0xFF stand for a pair of bytes (each again plain
ASCII or a pair), out() in pf_io.c expands them. The table is written as pfdict.c (2 bytes per pair).

Encoded are only literals that are printed or compared by eq() and nothing else: arguments of the output
functions and eq() (ENCODED), literals wrapped in T(...) (tables of messages, see pf.h) and the config
names of cfgnames.h (F(index, pin, "name"), compared with eq()). Literals that are copied or indexed
(lineStart() prefixes, char arrays, strcmp()) stay plain. Messages must be ASCII.

  pfstrings.py <outdir> <file.c|cfgnames.h>...      writes <outdir>/<file>... and <outdir>/pfdict.c
"""
import os
import re
import sys

ENCODED = {"out", "note", "warn", "err", "kv", "kvi", "kvp", "kvs", "kvmv", "heading", "T", "eq"}
BASE = 0x80
MAXPAIRS = 128

ESC = {"n": 10, "r": 13, "t": 9, "a": 7, "b": 8, "f": 12, "v": 11, "\\": 92, "'": 39, '"': 34, "?": 63}


def unescape(body):
    out, i = [], 0
    while i < len(body):
        c = body[i]
        if c != "\\":
            out.append(ord(c))
            i += 1
            continue
        n = body[i + 1]
        if n in ESC:
            out.append(ESC[n])
            i += 2
        elif n == "x":
            m = re.match(r"[0-9a-fA-F]+", body[i + 2:])
            out.append(int(m.group(0), 16) & 0xFF)
            i += 2 + len(m.group(0))
        elif n in "01234567":
            m = re.match(r"[0-7]{1,3}", body[i + 1:])
            out.append(int(m.group(0), 8))
            i += 1 + len(m.group(0))
        else:
            raise SystemExit("pfstrings: unknown escape \\" + n)
    return out


def escape(data):
    s = []
    for b in data:
        if b == 34:
            s.append('\\"')
        elif b == 92:
            s.append("\\\\")
        elif 32 <= b < 127 and b != 63:    # '?' as octal: no trigraph surprises
            s.append(chr(b))
        else:
            s.append("\\%03o" % b)    # always 3 digits: never merges with the next character
    return '"' + "".join(s) + '"'


TOKEN = re.compile(r'''
    (?P<comment>//[^\n]*|/\*.*?\*/)
  | (?P<pp>^[ \t]*\#(?:[^\n\\]|\\.)*)
  | (?P<str>"(?:[^"\\\n]|\\.)*")
  | (?P<chr>'(?:[^'\\\n]|\\.)*')
  | (?P<id>[A-Za-z_]\w*)
  | (?P<open>\()
  | (?P<close>\))
  | (?P<other>.)
''', re.S | re.M | re.X)


def scan(src, header=False):
    """yields (start, end, bytes) of the literals to encode"""
    if header:    # cfgnames.h: the names in F(index, pin, "name") (inside a #define)
        for m in re.finditer(r'\bF\(\s*\d+\s*,\s*\d\s*,\s*("\w*")', src):
            yield m.start(1), m.end(1), unescape(m.group(1)[1:-1])
        return
    stack, last_id = [], None
    for m in TOKEN.finditer(src):
        k = m.lastgroup
        if k in ("comment", "pp", "chr"):
            continue
        if k == "id":
            last_id = m.group(0)
            continue
        if k == "open":
            stack.append(last_id)
        elif k == "close":
            if stack:
                stack.pop()
        elif k == "str" and stack and stack[-1] in ENCODED:
            data = unescape(m.group(0)[1:-1])
            if any(b >= 0x80 for b in data):
                raise SystemExit("pfstrings: non-ASCII message " + m.group(0))
            yield m.start(), m.end(), data
        if k != "other" or not m.group(0).isspace():
            last_id = None


def bpe(strings):
    """pairs [(a, b)] and the encoded strings; a pair never contains 0 (the terminator)"""
    seqs = [list(s) for s in sorted(set(map(bytes, strings)))]    # sorted: the same table every build
    pairs = []
    while len(pairs) < MAXPAIRS:
        count = {}
        for s in seqs:
            prev = None
            for i in range(len(s) - 1):
                p = (s[i], s[i + 1])
                if 0 in p or (p == prev and s[i] == s[i + 1]):    # "aaa": count overlaps once
                    prev = None
                    continue
                count[p] = count.get(p, 0) + 1
                prev = p
        if not count:
            break
        p, n = max(sorted(count.items()), key=lambda kv: kv[1])
        if n < 3:    # saves n bytes, costs 2 in the table
            break
        sym = BASE + len(pairs)
        pairs.append(p)
        for s in seqs:
            i = 0
            while i < len(s) - 1:
                if (s[i], s[i + 1]) == p:
                    s[i:i + 2] = [sym]
                i += 1
    return pairs


def encode(data, pairs):
    s = list(data)
    for k, p in enumerate(pairs):
        i = 0
        while i < len(s) - 1:
            if (s[i], s[i + 1]) == p:
                s[i:i + 2] = [BASE + k]
            i += 1
    return s


def decode(s, pairs):
    out = []
    for b in s:
        stack = [b]
        while stack:
            c = stack.pop()
            if c >= BASE:
                stack += reversed(pairs[c - BASE])
            else:
                out.append(c)
    return out


def main():
    outdir, files = sys.argv[1], sys.argv[2:]
    srcs = {f: open(f, encoding="utf-8").read() for f in files}
    found = {f: list(scan(s, f.endswith(".h"))) for f, s in srcs.items()}
    pairs = bpe([d for lits in found.values() for _, _, d in lits])
    before = after = 0
    seen = set()
    os.makedirs(outdir, exist_ok=True)
    for f, src in srcs.items():
        parts, pos = [], 0
        for a, b, data in found[f]:
            enc = encode(data, pairs)
            assert decode(enc, pairs) == data
            if bytes(data) not in seen:
                seen.add(bytes(data))
                before += len(data) + 1
                after += len(enc) + 1
            parts += [src[pos:a], escape(enc)]
            pos = b
        parts.append(src[pos:])
        dst = os.path.join(outdir, os.path.basename(f))
        text = '#line 1 "%s"\n' % f + "".join(parts)
        if not os.path.exists(dst) or open(dst, encoding="utf-8").read() != text:
            open(dst, "w", encoding="utf-8").write(text)
    rows = ",\n".join("\t{ %d, %d }" % p for p in pairs) or "\t{ 0, 0 }"
    dict_c = ("//generated by tools/pfstrings.py: the byte pairs of PinFinder's messages (pf_io.c out())\n"
              "#include <stdint.h>\n"
              "const uint8_t pfPairs[][2] = {\n%s\n};\n" % rows)
    dst = os.path.join(outdir, "pfdict.c")
    if not os.path.exists(dst) or open(dst).read() != dict_c:
        open(dst, "w").write(dict_c)
    print("pfstrings: %d messages %d -> %d bytes + %d pair table" % (len(seen), before, after, 2 * len(pairs)))


if __name__ == "__main__":
    main()
