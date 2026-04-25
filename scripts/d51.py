#!/usr/bin/env python3
"""Minimal 8051 (MCS-51) disassembler for the EdgeTPU apex firmware.

Reference: Intel 8051 ISA opcode table (256 opcodes).
Usage:  ./d51.py firmware.bin  [--start 0x0000] [--end 0xFFFF]
Output: one line per instruction `ADDR: BYTES   MNEMONIC OPERANDS`
"""
import sys, argparse

# Canonical SFR names at direct addresses (0x80-0xFF)
SFR = {
    0x80:'P0', 0x81:'SP', 0x82:'DPL', 0x83:'DPH', 0x87:'PCON',
    0x88:'TCON', 0x89:'TMOD', 0x8A:'TL0', 0x8B:'TL1', 0x8C:'TH0', 0x8D:'TH1',
    0x90:'P1',
    0x98:'SCON', 0x99:'SBUF',
    0xA0:'P2', 0xA8:'IE', 0xA9:'SADDR',
    0xB0:'P3', 0xB8:'IP', 0xB9:'SADEN',
    0xC8:'T2CON', 0xC9:'T2MOD', 0xCA:'RCAP2L', 0xCB:'RCAP2H',
    0xCC:'TL2', 0xCD:'TH2',
    0xD0:'PSW',
    0xE0:'ACC',
    0xF0:'B',
}

# Register-bit names for bit-addressable SFRs (0x80-0xFF, bit addresses 0x80-0xFF)
# Bit address in 0x00-0x7F = RAM bit-addressable (banks 0x20-0x2F)
# Bit address in 0x80-0xFF = SFR bit (byte = (bit&0xF8); bit_in_byte = bit & 7)
def bit_name(b):
    if b < 0x80:
        byte = 0x20 + (b >> 3)
        return f'20.{b&7}({byte:02X}.{b&7})'
    byte = b & 0xF8
    sfr = SFR.get(byte, f'{byte:02X}H')
    return f'{sfr}.{b&7}'

def dir_name(d):
    if d >= 0x80:
        return SFR.get(d, f'{d:02X}H')
    # 0x00-0x1F = register banks 0-3 (R0..R7 × 4)
    if d < 0x20:
        bank, reg = divmod(d, 8)
        return f'R{reg}#bank{bank}({d:02X}H)'
    return f'{d:02X}H'

# Opcode table: each entry is (mnemonic_fmt, byte_count, operand_fmt)
# operand_fmt is a lambda (pc, operands) -> string.
# mnemonic_fmt can include placeholders {a1}, {a2}.

def signed(b):
    return b - 256 if b >= 128 else b

def _rel(pc, b):
    return (pc + signed(b)) & 0xFFFF

OPS = [None] * 256

# Fill in opcodes.  Most grouping follows the 8051 ISA pattern.
def op(code, mnemonic, n, fmt):
    OPS[code] = (mnemonic, n, fmt)

op(0x00, 'NOP', 1, lambda pc,o: '')
# LJMP addr16 / LCALL addr16
op(0x02, 'LJMP', 3, lambda pc,o: f'{(o[0]<<8)|o[1]:04X}H')
op(0x12, 'LCALL', 3, lambda pc,o: f'{(o[0]<<8)|o[1]:04X}H')
op(0x22, 'RET', 1, lambda pc,o: '')
op(0x32, 'RETI', 1, lambda pc,o: '')

# AJMP addr11 — opcode = a10-a8 << 5 | 0x01
# ACALL addr11 — opcode = a10-a8 << 5 | 0x11
for hi in range(8):
    code_j = (hi << 5) | 0x01
    code_c = (hi << 5) | 0x11
    def _ajmp(pc,o,hi=hi):
        target = ((pc) & 0xF800) | (hi << 8) | o[0]
        return f'{target:04X}H'
    op(code_j, 'AJMP', 2, _ajmp)
    op(code_c, 'ACALL', 2, _ajmp)

op(0x03, 'RR A', 1, lambda pc,o: '')
op(0x13, 'RRC A', 1, lambda pc,o: '')
op(0x23, 'RL A', 1, lambda pc,o: '')
op(0x33, 'RLC A', 1, lambda pc,o: '')

op(0x04, 'INC A', 1, lambda pc,o: '')
op(0x05, 'INC', 2, lambda pc,o: dir_name(o[0]))
op(0x06, 'INC @R0', 1, lambda pc,o: '')
op(0x07, 'INC @R1', 1, lambda pc,o: '')
for r in range(8):
    op(0x08+r, f'INC R{r}', 1, lambda pc,o: '')

op(0x14, 'DEC A', 1, lambda pc,o: '')
op(0x15, 'DEC', 2, lambda pc,o: dir_name(o[0]))
op(0x16, 'DEC @R0', 1, lambda pc,o: '')
op(0x17, 'DEC @R1', 1, lambda pc,o: '')
for r in range(8):
    op(0x18+r, f'DEC R{r}', 1, lambda pc,o: '')

op(0xA3, 'INC DPTR', 1, lambda pc,o: '')

# ADD/ADDC/SUBB/ORL/ANL/XRL
def arith(base, mnem):
    op(base+0x04, f'{mnem} A,#imm',    2, lambda pc,o: f'A,#{o[0]:02X}H')
    op(base+0x05, f'{mnem} A,dir',     2, lambda pc,o: f'A,{dir_name(o[0])}')
    op(base+0x06, f'{mnem} A,@R0',     1, lambda pc,o: '')
    op(base+0x07, f'{mnem} A,@R1',     1, lambda pc,o: '')
    for r in range(8):
        op(base+0x08+r, f'{mnem} A,R{r}', 1, lambda pc,o: '')

arith(0x20, 'ADD')   # 0x24..0x2F
arith(0x30, 'ADDC')  # 0x34..0x3F
arith(0x90, 'SUBB')  # 0x94..0x9F
# ANL / ORL / XRL have more complex forms
for base, mnem in [(0x50,'ANL'), (0x40,'ORL'), (0x60,'XRL')]:
    op(base+0x02, f'{mnem} dir,A',    2, lambda pc,o: f'{dir_name(o[0])},A')
    op(base+0x03, f'{mnem} dir,#imm', 3, lambda pc,o: f'{dir_name(o[0])},#{o[1]:02X}H')
    op(base+0x04, f'{mnem} A,#imm',   2, lambda pc,o: f'A,#{o[0]:02X}H')
    op(base+0x05, f'{mnem} A,dir',    2, lambda pc,o: f'A,{dir_name(o[0])}')
    op(base+0x06, f'{mnem} A,@R0',    1, lambda pc,o: '')
    op(base+0x07, f'{mnem} A,@R1',    1, lambda pc,o: '')
    for r in range(8):
        op(base+0x08+r, f'{mnem} A,R{r}', 1, lambda pc,o: '')

# ANL C,bit / ORL C,bit / ANL C,/bit / ORL C,/bit
op(0x82, 'ANL C,bit', 2, lambda pc,o: f'C,{bit_name(o[0])}')
op(0xB0, 'ANL C,/bit',2, lambda pc,o: f'C,/{bit_name(o[0])}')
op(0x72, 'ORL C,bit', 2, lambda pc,o: f'C,{bit_name(o[0])}')
op(0xA0, 'ORL C,/bit',2, lambda pc,o: f'C,/{bit_name(o[0])}')

# MOV family — a subset
op(0x74, 'MOV A,#imm', 2, lambda pc,o: f'A,#{o[0]:02X}H')
op(0xE5, 'MOV A,dir',  2, lambda pc,o: f'A,{dir_name(o[0])}')
op(0xE6, 'MOV A,@R0',  1, lambda pc,o: '')
op(0xE7, 'MOV A,@R1',  1, lambda pc,o: '')
for r in range(8):
    op(0xE8+r, f'MOV A,R{r}', 1, lambda pc,o: '')

op(0x76, 'MOV @R0,#imm', 2, lambda pc,o: f'@R0,#{o[0]:02X}H')
op(0x77, 'MOV @R1,#imm', 2, lambda pc,o: f'@R1,#{o[0]:02X}H')
for r in range(8):
    op(0x78+r, f'MOV R{r},#imm', 2, lambda pc,o: f'R{r},#{o[0]:02X}H')

op(0xF5, 'MOV dir,A',    2, lambda pc,o: f'{dir_name(o[0])},A')
op(0x85, 'MOV dir,dir',  3, lambda pc,o: f'{dir_name(o[1])},{dir_name(o[0])}')
op(0x86, 'MOV dir,@R0',  2, lambda pc,o: f'{dir_name(o[0])},@R0')
op(0x87, 'MOV dir,@R1',  2, lambda pc,o: f'{dir_name(o[0])},@R1')
for r in range(8):
    op(0x88+r, f'MOV dir,R{r}', 2, lambda pc,o: f'{dir_name(o[0])},R{r}')

op(0x75, 'MOV dir,#imm', 3, lambda pc,o: f'{dir_name(o[0])},#{o[1]:02X}H')

op(0xF6, 'MOV @R0,A', 1, lambda pc,o: '')
op(0xF7, 'MOV @R1,A', 1, lambda pc,o: '')
for r in range(8):
    op(0xF8+r, f'MOV R{r},A', 1, lambda pc,o: '')

op(0xA6, 'MOV @R0,dir', 2, lambda pc,o: f'@R0,{dir_name(o[0])}')
op(0xA7, 'MOV @R1,dir', 2, lambda pc,o: f'@R1,{dir_name(o[0])}')
for r in range(8):
    op(0xA8+r, f'MOV R{r},dir', 2, lambda pc,o: f'R{r},{dir_name(o[0])}')

op(0x90, 'MOV DPTR,#imm16', 3, lambda pc,o: f'DPTR,#{(o[0]<<8)|o[1]:04X}H')

# MOVC / MOVX
op(0x83, 'MOVC A,@A+PC',   1, lambda pc,o: '')
op(0x93, 'MOVC A,@A+DPTR', 1, lambda pc,o: '')
op(0xE0, 'MOVX A,@DPTR',   1, lambda pc,o: '')
op(0xE2, 'MOVX A,@R0',     1, lambda pc,o: '')
op(0xE3, 'MOVX A,@R1',     1, lambda pc,o: '')
op(0xF0, 'MOVX @DPTR,A',   1, lambda pc,o: '')
op(0xF2, 'MOVX @R0,A',     1, lambda pc,o: '')
op(0xF3, 'MOVX @R1,A',     1, lambda pc,o: '')

# SETB / CLR / CPL
op(0xD2, 'SETB bit', 2, lambda pc,o: bit_name(o[0]))
op(0xD3, 'SETB C',   1, lambda pc,o: '')
op(0xC2, 'CLR bit',  2, lambda pc,o: bit_name(o[0]))
op(0xC3, 'CLR C',    1, lambda pc,o: '')
op(0xE4, 'CLR A',    1, lambda pc,o: '')
op(0xB2, 'CPL bit',  2, lambda pc,o: bit_name(o[0]))
op(0xB3, 'CPL C',    1, lambda pc,o: '')
op(0xF4, 'CPL A',    1, lambda pc,o: '')

# JB / JNB / JBC bit,rel
op(0x20, 'JB',  3, lambda pc,o: f'{bit_name(o[0])},{_rel(pc,o[1]):04X}H')
op(0x30, 'JNB', 3, lambda pc,o: f'{bit_name(o[0])},{_rel(pc,o[1]):04X}H')
op(0x10, 'JBC', 3, lambda pc,o: f'{bit_name(o[0])},{_rel(pc,o[1]):04X}H')

# JC / JNC / JZ / JNZ / SJMP
op(0x40, 'JC',   2, lambda pc,o: f'{_rel(pc,o[0]):04X}H')
op(0x50, 'JNC',  2, lambda pc,o: f'{_rel(pc,o[0]):04X}H')
op(0x60, 'JZ',   2, lambda pc,o: f'{_rel(pc,o[0]):04X}H')
op(0x70, 'JNZ',  2, lambda pc,o: f'{_rel(pc,o[0]):04X}H')
op(0x80, 'SJMP', 2, lambda pc,o: f'{_rel(pc,o[0]):04X}H')

# CJNE / DJNZ
op(0xB4, 'CJNE A,#imm,rel', 3, lambda pc,o: f'A,#{o[0]:02X}H,{_rel(pc,o[1]):04X}H')
op(0xB5, 'CJNE A,dir,rel',  3, lambda pc,o: f'A,{dir_name(o[0])},{_rel(pc,o[1]):04X}H')
op(0xB6, 'CJNE @R0,#imm,rel',3, lambda pc,o: f'@R0,#{o[0]:02X}H,{_rel(pc,o[1]):04X}H')
op(0xB7, 'CJNE @R1,#imm,rel',3, lambda pc,o: f'@R1,#{o[0]:02X}H,{_rel(pc,o[1]):04X}H')
for r in range(8):
    op(0xB8+r, f'CJNE R{r},#imm,rel', 3, lambda pc,o: f'R{r},#{o[0]:02X}H,{_rel(pc,o[1]):04X}H')

op(0xD5, 'DJNZ dir,rel', 3, lambda pc,o: f'{dir_name(o[0])},{_rel(pc,o[1]):04X}H')
for r in range(8):
    op(0xD8+r, f'DJNZ R{r},rel', 2, lambda pc,o: f'R{r},{_rel(pc,o[0]):04X}H')

# PUSH / POP / XCH / XCHD
op(0xC0, 'PUSH', 2, lambda pc,o: dir_name(o[0]))
op(0xD0, 'POP',  2, lambda pc,o: dir_name(o[0]))
op(0xC5, 'XCH A,dir',  2, lambda pc,o: f'A,{dir_name(o[0])}')
op(0xC6, 'XCH A,@R0',  1, lambda pc,o: '')
op(0xC7, 'XCH A,@R1',  1, lambda pc,o: '')
for r in range(8):
    op(0xC8+r, f'XCH A,R{r}', 1, lambda pc,o: '')
op(0xD6, 'XCHD A,@R0', 1, lambda pc,o: '')
op(0xD7, 'XCHD A,@R1', 1, lambda pc,o: '')

op(0x84, 'DIV AB', 1, lambda pc,o: '')
op(0xA4, 'MUL AB', 1, lambda pc,o: '')
op(0xD4, 'DA A',   1, lambda pc,o: '')
op(0xA2, 'MOV C,bit', 2, lambda pc,o: f'C,{bit_name(o[0])}')
op(0x92, 'MOV bit,C', 2, lambda pc,o: f'{bit_name(o[0])},C')
op(0x73, 'JMP @A+DPTR', 1, lambda pc,o: '')

# reserved / undefined
op(0xA5, 'UNDEF A5', 1, lambda pc,o: '')

def disasm(buf, start=0, end=None):
    if end is None: end = len(buf)
    pc = start
    out = []
    while pc < end:
        b = buf[pc]
        entry = OPS[b]
        if entry is None:
            out.append((pc, [b], f'DB {b:02X}H', '?undef'))
            pc += 1
            continue
        mnem, n, fmt = entry
        if pc + n > end:
            out.append((pc, [b], f'DB {b:02X}H (partial)', ''))
            pc += 1
            continue
        operands = [buf[pc+1+i] for i in range(n-1)]
        # pc for relative jumps is the address AFTER this instruction
        ops = fmt(pc + n, operands)
        out.append((pc, [b]+operands, mnem, ops))
        pc += n
    return out

def fmt_line(pc, bytes_, mnem, ops):
    bs = ' '.join(f'{b:02X}' for b in bytes_).ljust(10)
    txt = mnem if not ops else f'{mnem} {ops}'
    return f'{pc:04X}:  {bs}  {txt}'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('file')
    ap.add_argument('--start', type=lambda x: int(x,0), default=0)
    ap.add_argument('--end',   type=lambda x: int(x,0), default=None)
    a = ap.parse_args()
    with open(a.file,'rb') as f: buf = f.read()
    end = a.end if a.end is not None else len(buf)
    for pc, bs, mnem, ops in disasm(buf, a.start, end):
        print(fmt_line(pc, bs, mnem, ops))

if __name__ == '__main__':
    main()
