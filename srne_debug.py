#!/usr/bin/env python3
"""
srne_debug.py - Script de diagnosticare conexiune SRNE Modbus RTU
==================================================================
Rulare din terminal SSH pe HA sau orice Linux cu Python 3:
  pip install pyserial
  python3 srne_debug.py [port] [--scan] [--reg 0x0100 35] [--write 0xE208 2300]

Exemple:
  python3 srne_debug.py /dev/ttyUSB1              # test complet
  python3 srne_debug.py --scan                    # scanare porturi
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 35   # citire bloc baterie
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1    # citire machine state
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300  # scriere registru
  python3 srne_debug.py /dev/ttyUSB1 --raw        # dump hex brut
  python3 srne_debug.py /dev/ttyUSB1 --baud 9600  # viteza alternativa

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import sys
import time
import struct
import argparse
import glob
import os

try:
    import serial
except ImportError:
    print("Lipsa: pip install pyserial")
    sys.exit(1)

# --- CRC16 Modbus -------------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def build_fc03(addr, reg_start, count):
    pdu = struct.pack(">BBHH", addr, 0x03, reg_start, count)
    return pdu + struct.pack("<H", crc16(pdu))

def build_fc06(addr, reg, value):
    pdu = struct.pack(">BBHH", addr, 0x06, reg, value & 0xFFFF)
    return pdu + struct.pack("<H", crc16(pdu))

# --- Comunicare serial --------------------------------------------------------

def transact(ser, request, expected_bytes, timeout=2.0):
    ser.reset_input_buffer()
    ser.write(request)
    print(f"  TX ({len(request)}b): {request.hex(' ').upper()}")

    resp = bytearray()
    start = time.time()
    while len(resp) < expected_bytes and time.time() - start < timeout:
        chunk = ser.read(expected_bytes - len(resp))
        if chunk:
            resp.extend(chunk)

    print(f"  RX ({len(resp)}b): {resp.hex(' ').upper() if resp else '(nimic)'}")

    if len(resp) < expected_bytes:
        print(f"  WARNING Raspuns scurt: {len(resp)}/{expected_bytes} bytes")
        return None

    crc_recv = struct.unpack("<H", bytes(resp[-2:]))[0]
    crc_calc = crc16(bytes(resp[:-2]))
    if crc_recv != crc_calc:
        print(f"  FAIL CRC: primit=0x{crc_recv:04X} calculat=0x{crc_calc:04X}")
        return None

    if resp[1] & 0x80:
        exc = resp[2] if len(resp) > 2 else 0
        exc_map = {1: "FC nesuportat", 2: "Adresa invalida", 3: "Date prea mari", 4: "Eroare citire"}
        print(f"  FAIL Modbus exception: {exc_map.get(exc, f'cod {exc}')}")
        return None

    print(f"  OK")
    return bytes(resp)

def read_regs(ser, addr, reg_start, count):
    resp = transact(ser, build_fc03(addr, reg_start, count), 3 + count * 2 + 2)
    if resp is None:
        return None
    return [struct.unpack(">H", resp[3+i*2:5+i*2])[0] for i in range(count)]

def write_reg(ser, addr, reg, value):
    resp = transact(ser, build_fc06(addr, reg, value), 8)
    return resp is not None

# --- Parsare SRNE -------------------------------------------------------------

CHARGE_STATE  = {0:"Off",1:"Active",2:"MPPT",3:"Equalizing",4:"Boost",5:"Float",6:"Current limit"}
MACHINE_STATE = {0:"Standby",1:"No anomaly",2:"SW startup",3:"Starting",
                 4:"Line mode",5:"Inverter mode",6:"ECO mode",
                 7:"Fault",8:"Shutdown",9:"Running (inverter)"}

def parse_0100(regs):
    def r(a): return regs[a - 0x0100]
    t = r(0x0103)
    tc = (t >> 8) & 0x7F
    tb = t & 0x7F
    cs = r(0x010C) & 0xFF
    fault = (r(0x0121) << 16) | r(0x0122)
    print("\n  +-- Bloc 0x0100 - Baterie + PV ----------------------------")
    print(f"  |  SOC:           {r(0x0100) & 0xFF}%")
    print(f"  |  Vbat:          {r(0x0101) * 0.1:.1f} V  (raw: {r(0x0101):#06x})")
    print(f"  |  Ibat:          {r(0x0102) * 0.1:.1f} A  (raw: {r(0x0102):#06x})")
    print(f"  |  Temp ctrl:     {tc}C  | Temp bat: {tb}C")
    print(f"  |  Vpv:           {r(0x0107) * 0.1:.1f} V")
    print(f"  |  Ipv:           {r(0x0108) * 0.01:.2f} A")
    print(f"  |  Ppv:           {r(0x0109)} W")
    print(f"  |  Charge state:  {CHARGE_STATE.get(cs, f'?({cs})')}  (raw: {cs})")
    print(f"  |  Fault word:    0x{fault:08X}  {'OK' if fault == 0 else 'FAULT!'}")
    print(f"  +----------------------------------------------------------")

def parse_0204(regs):
    def r(a):
        i = a - 0x0204
        return regs[i] if 0 <= i < len(regs) else 0
    ms  = r(0x0209) & 0xFF
    r0, r1, r2 = r(0x020C), r(0x020D), r(0x020E)
    pac = r(0x021B)
    pap = r(0x021C)
    print(f"\n  +-- Bloc 0x0204 - Iesire AC + Temps -----------------------")
    print(f"  |  Machine state: {MACHINE_STATE.get(ms, f'?({ms})')}  (raw: {ms})")
    print(f"  |  RTC:           {(r0>>8)+2002}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}")
    print(f"  |  Load ratio:    {r(0x0210)}%")
    print(f"  |  Vac out:       {r(0x0216) * 0.1:.1f} V")
    print(f"  |  Fac out:       {r(0x0218) * 0.01:.2f} Hz")
    print(f"  |  Iac out:       {r(0x0219) * 0.1:.1f} A")
    print(f"  |  Pac active:    {pac} W")
    print(f"  |  Pac apparent:  {pap} VA")
    print(f"  |  Power factor:  {round(pac/pap,3) if pap else 0.0:.3f}")
    print(f"  |  Temp DC side:  {r(0x0220) * 0.1:.1f}C")
    print(f"  |  Temp AC side:  {r(0x0221) * 0.1:.1f}C")
    print(f"  |  Temp trafo:    {r(0x0222) * 0.1:.1f}C")
    print(f"  +----------------------------------------------------------")

def parse_F02F(regs):
    def r(a):
        i = a - 0xF02F
        return regs[i] if 0 <= i < len(regs) else 0
    print(f"\n  +-- Bloc 0xF02F - Energie ---------------------------------")
    print(f"  |  PV azi:        {r(0xF02F) * 0.1:.1f} kWh")
    print(f"  |  Sarcina azi:   {r(0xF030) * 0.1:.1f} kWh")
    print(f"  |  PV total:      {r(0xF038) * 0.1:.1f} kWh")
    print(f"  |  Sarcina total: {r(0xF03A) * 0.1:.1f} kWh")
    print(f"  +----------------------------------------------------------")

# --- Scanare porturi ----------------------------------------------------------

def scan_ports():
    print("\n=== Scanare porturi seriale ===================================")
    by_id_dir = "/dev/serial/by-id/"
    if os.path.isdir(by_id_dir):
        links = sorted(glob.glob(by_id_dir + "*"))
        if links:
            print("\n  /dev/serial/by-id/:")
            for link in links:
                print(f"    {os.path.basename(link)}")
                print(f"      -> {os.path.realpath(link)}")
        else:
            print("  /dev/serial/by-id/ gol")
    else:
        print("  /dev/serial/by-id/ indisponibil (nu esti pe Linux?)")
    tty_list = sorted(glob.glob("/dev/ttyUSB*"))
    print(f"\n  ttyUSB: {', '.join(tty_list) if tty_list else 'niciun'}")
    print()

# --- Main ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SRNE Modbus RTU Debug Tool")
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB1")
    parser.add_argument("--addr",  type=lambda x: int(x, 0), default=1)
    parser.add_argument("--baud",  type=int, default=9600)
    parser.add_argument("--scan",  action="store_true")
    parser.add_argument("--reg",   nargs=2, metavar=("START", "COUNT"))
    parser.add_argument("--write", nargs=2, metavar=("REG", "VALUE"))
    parser.add_argument("--raw",   action="store_true")
    args = parser.parse_args()

    print("=" * 60)
    print("  SRNE Invertor - Modbus RTU Debug Tool")
    print("=" * 60)

    scan_ports()

    if args.scan and not args.reg and not args.write:
        return

    print(f"  Port: {args.port} | Baud: {args.baud} 8N1 | Addr: {args.addr}\n")

    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8,
                            parity=serial.PARITY_NONE, stopbits=1, timeout=1.0)
        print(f"  Port deschis OK: {args.port}\n")
    except serial.SerialException as e:
        print(f"  FAIL Nu pot deschide {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.2)

    if args.reg:
        reg_start = int(args.reg[0], 0)
        count = int(args.reg[1])
        print(f"=== Citire 0x{reg_start:04X} x {count} registri ========================")
        regs = read_regs(ser, args.addr, reg_start, count)
        if regs:
            print("\n  Valori (decimal | hex | binar):")
            for i, v in enumerate(regs):
                print(f"    [0x{reg_start+i:04X}]  {v:>6}  0x{v:04X}  {v:016b}b")
        ser.close()
        return

    if args.write:
        reg = int(args.write[0], 0)
        val = int(args.write[1], 0)
        print(f"=== Scriere 0x{reg:04X} = {val} (0x{val:04X}) =========================")
        ok = write_reg(ser, args.addr, reg, val)
        print(f"\n  Rezultat: {'OK - Succes' if ok else 'FAIL - Esuat'}")
        ser.close()
        return

    # Test complet
    print("=== Test complet - toate blocurile ===========================\n")

    print("Citire bloc 0x0100 (35 registri - baterie + PV + fault)...")
    r0100 = read_regs(ser, args.addr, 0x0100, 35)
    if r0100:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in r0100])
        else: parse_0100(r0100)
    time.sleep(0.15)

    print("\nCitire bloc 0x0204 (31 registri - AC output + temps + RTC)...")
    r0204 = read_regs(ser, args.addr, 0x0204, 31)
    if r0204:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in r0204])
        else: parse_0204(r0204)
    time.sleep(0.15)

    print("\nCitire bloc 0xF02F (13 registri - energie)...")
    rF02F = read_regs(ser, args.addr, 0xF02F, 13)
    if rF02F:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in rF02F])
        else: parse_F02F(rF02F)
    time.sleep(0.15)

    print("\nCitire E-registri...")
    for reg, desc in [(0xE004, "Machine state"), (0xE204, "Fault/alarm")]:
        print(f"\n  {desc} (0x{reg:04X}):")
        regs = read_regs(ser, args.addr, reg, 1)
        if regs:
            v = regs[0]
            if reg == 0xE004:
                print(f"    {v} -> {MACHINE_STATE.get(v, f'Unknown({v})')}")
            else:
                print(f"    {v} -> {'OK' if v == 0 else f'FAULT 0x{v:04X}'}")

    print("\n=== Test complet finalizat ===================================\n")
    ser.close()

if __name__ == "__main__":
    main()
