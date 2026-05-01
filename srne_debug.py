#!/usr/bin/env python3
"""
srne_debug.py v1.4 - Diagnosticare SRNE Invertor Modbus RTU
============================================================
Rulare din terminal SSH pe HA:
  pip install pyserial
  python3 /config/addons/srne_invertor/srne_debug.py /dev/ttyUSB1

Output salvat in acelasi folder cu scriptul: srne_debug.log

Exemple:
  python3 srne_debug.py /dev/ttyUSB1              # test complet
  python3 srne_debug.py --scan                    # scanare porturi
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15  # citire registri
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1   # machine state
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300  # scriere registru
  python3 srne_debug.py /dev/ttyUSB1 --raw        # dump hex brut

Arhitectura bus:
  Portul USB-B al invertorului prezinta o interfata Modbus SLAVE (addr=1)
  pentru monitorizare externa. Scriptul este MASTER care interogheaza
  invertorul. Receive-ul filtreaza raspunsurile dupa addr=0x01 + CRC.

Changelog:
  v1.4 - Ibat interpretat ca signed int16 (negativ=incarcare)
         eliminat 0x0113 si 0x0121 din test complet (nu exista pe HF2450S80H)
  v1.3 - log relativ la folder script
  v1.2 - log in fisier, filtrare dupa adresa slave
  v1.1 - receive inteligent, citire 0x0100 x 15 regs
  v1.0 - initial

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import sys
import time
import struct
import argparse
import glob
import os
import logging
from datetime import datetime

try:
    import serial
except ImportError:
    print("Lipsa: pip install pyserial")
    sys.exit(1)

# --- Logger -------------------------------------------------------------------

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "srne_debug.log")

def setup_logger():
    logger = logging.getLogger("srne")
    logger.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S")
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    logger.addHandler(ch)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt)
    logger.addHandler(fh)
    return logger

log = setup_logger()

def p(msg=""):
    log.info(msg)

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

def to_signed16(val: int) -> int:
    """uint16 → int16 cu semn (two's complement). Folosit pentru Ibat."""
    return val if val < 0x8000 else val - 0x10000

# --- Receive cu filtrare dupa adresa slave ------------------------------------

def recv_slave_frame(ser, slave_addr, expected_regs, timeout=3.0):
    """
    Citeste bytes de pe bus si cauta un frame Modbus valid pentru slave_addr.
    Orice byte cu alta adresa este ignorat (inclusiv traficul invertor->BMS).

    Frame normal FC03:  [addr][0x03][byte_count][data...][CRC_L][CRC_H]
    Frame exception:    [addr][0x83][exc_code][CRC_L][CRC_H]
    """
    buf = bytearray()
    start = time.time()
    ignored_bytes = 0

    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

        i = 0
        while i < len(buf):
            b = buf[i]
            if b != slave_addr:
                ignored_bytes += 1
                i += 1
                continue

            rest = buf[i:]

            # Raspuns normal FC03
            if len(rest) >= 3 and rest[1] == 0x03:
                byte_count = rest[2]
                if byte_count != expected_regs * 2:
                    i += 1
                    continue
                total = 3 + byte_count + 2
                if len(rest) < total:
                    break
                frame = bytes(rest[:total])
                if struct.unpack("<H", frame[-2:])[0] == crc16(frame[:-2]):
                    if ignored_bytes > 0:
                        p(f"  [bus] ignorat {ignored_bytes}b trafic de la alte dispozitive")
                    return frame, False
                i += 1
                continue

            # Raspuns exception FC03 (0x83)
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == crc16(frame[:-2]):
                    if ignored_bytes > 0:
                        p(f"  [bus] ignorat {ignored_bytes}b trafic de la alte dispozitive")
                    return frame, True
                i += 1
                continue

            i += 1

    if ignored_bytes > 0:
        p(f"  [bus] ignorat {ignored_bytes}b (timeout)")
    return None, False


def transact(ser, slave_addr, request, expected_regs, timeout=3.0):
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.write(request)
    p(f"  TX ({len(request)}b): {request.hex(' ').upper()}")

    frame, is_exc = recv_slave_frame(ser, slave_addr, expected_regs, timeout)

    if frame is None:
        p(f"  RX: timeout - niciun raspuns de la addr=0x{slave_addr:02X}")
        return None

    p(f"  RX ({len(frame)}b): {frame.hex(' ').upper()}")

    if is_exc:
        exc_code = frame[2]
        exc_map = {1: "FC nesuportat", 2: "Adresa/count invalid",
                   3: "Count prea mare", 4: "Eroare interna",
                   10: "Depasire limita bloc PDU"}
        p(f"  FAIL Exception 0x{exc_code:02X}: {exc_map.get(exc_code, f'cod {exc_code}')}")
        return None

    p(f"  OK")
    return frame


def read_regs(ser, slave_addr, reg_start, count):
    frame = transact(ser, slave_addr, build_fc03(slave_addr, reg_start, count), count)
    if frame is None:
        return None
    return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(count)]


def write_reg(ser, slave_addr, reg, value):
    request = build_fc06(slave_addr, reg, value)
    ser.reset_input_buffer()
    ser.write(request)
    p(f"  TX ({len(request)}b): {request.hex(' ').upper()}")
    resp = bytearray()
    start = time.time()
    while len(resp) < 8 and time.time() - start < 2.0:
        chunk = ser.read(8 - len(resp))
        if chunk:
            resp.extend(chunk)
    if len(resp) >= 8:
        p(f"  RX ({len(resp)}b): {bytes(resp[:8]).hex(' ').upper()}")
        crc_ok = struct.unpack("<H", bytes(resp[6:8]))[0] == crc16(bytes(resp[:6]))
        p(f"  {'OK' if crc_ok else 'FAIL CRC'}")
        return crc_ok
    p(f"  FAIL raspuns scurt ({len(resp)}b)")
    return False

# --- Parsare SRNE HF2450S80H --------------------------------------------------

CHARGE_STATE = {
    0: "Off", 1: "Active", 2: "MPPT", 3: "Equalizing",
    4: "Boost", 5: "Float", 6: "Current limit"
}
MACHINE_STATE = {
    0: "Standby", 1: "No anomaly", 2: "SW startup", 3: "Starting",
    4: "Line mode", 5: "Inverter mode", 6: "ECO mode",
    7: "Fault", 8: "Shutdown", 9: "Running (inverter)"
}


def parse_0100_15(regs):
    """
    Bloc 0x0100-0x010E (15 registri) — confirmat pe firmware HF2450S80H.
    Ibat este signed int16: negativ = incarcare baterie, pozitiv = descarcare.
    Vpv = tensiunea stringului PV (ex. 382V pentru 10 panouri in serie).
    """
    def r(a):
        i = a - 0x0100
        return regs[i] if 0 <= i < len(regs) else 0

    soc  = r(0x0100) & 0xFF
    vbat = r(0x0101) * 0.1
    ibat = to_signed16(r(0x0102)) * 0.1   # SIGNED: negativ=incarcare
    t    = r(0x0103)
    tc   = (t >> 8) & 0x7F
    tb   = t & 0x7F
    vpv  = r(0x0107) * 0.1
    ipv  = r(0x0108) * 0.01
    ppv  = r(0x0109)
    cs   = r(0x010C) & 0xFF

    ibat_dir = "incarcare" if ibat < 0 else "descarcare" if ibat > 0 else "standby"

    p("  +-- Bloc 0x0100-0x010E (baterie + string PV) -------------")
    p(f"  |  [0100] SOC:           {soc}%")
    p(f"  |  [0101] Vbat:          {vbat:.1f} V")
    p(f"  |  [0102] Ibat:          {ibat:.1f} A  ({ibat_dir})  raw: {r(0x0102):#06x}")
    p(f"  |  [0103] Temp ctrl:     {tc}C  | Temp bat: {tb}C")
    p(f"  |  [0107] Vpv string:    {vpv:.1f} V  (10 panouri serie)")
    p(f"  |  [0108] Ipv:           {ipv:.2f} A")
    p(f"  |  [0109] Ppv:           {ppv} W")
    p(f"  |  [010A] Load on/off:   {r(0x010A)}")
    p(f"  |  [010B] Vbat min azi:  {r(0x010B) * 0.1:.1f} V")
    p(f"  |  [010C] Charge step:   {CHARGE_STATE.get(cs, f'?({cs})')}  raw: {cs}")
    p("  +----------------------------------------------------------")


def parse_0204(regs):
    """
    Bloc 0x0204-0x0222 (31 registri) — specific firmware HF2450S80H.
    Contine starea invertorului, iesirea AC, RTC si temperaturi interne.
    """
    def r(a):
        i = a - 0x0204
        return regs[i] if 0 <= i < len(regs) else 0

    ms  = r(0x0209) & 0xFF
    r0, r1, r2 = r(0x020C), r(0x020D), r(0x020E)
    pac = r(0x021B)
    pap = r(0x021C)

    p("  +-- Bloc 0x0204-0x0222 (AC output + RTC + temperaturi) ---")
    p(f"  |  [0209] Machine state: {MACHINE_STATE.get(ms, f'?({ms})')}  raw: {ms}")
    p(f"  |  [020C-E] RTC:         {(r0>>8)+2002}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}")
    p(f"  |  [0210] Load ratio:    {r(0x0210)}%")
    p(f"  |  [0212] Running timer: {r(0x0212)} s")
    p(f"  |  [0216] Vac out:       {r(0x0216) * 0.1:.1f} V")
    p(f"  |  [0218] Fac out:       {r(0x0218) * 0.01:.2f} Hz")
    p(f"  |  [0219] Iac out:       {r(0x0219) * 0.1:.1f} A")
    p(f"  |  [021B] Pac activa:    {pac} W")
    p(f"  |  [021C] Pac aparenta:  {pap} VA")
    p(f"  |  [021B/C] PF:          {round(pac/pap, 3) if pap else 0.0:.3f}")
    p(f"  |  [0220] Temp DC side:  {r(0x0220) * 0.1:.1f} C")
    p(f"  |  [0221] Temp AC side:  {r(0x0221) * 0.1:.1f} C")
    p(f"  |  [0222] Temp trafo:    {r(0x0222) * 0.1:.1f} C")
    p("  +----------------------------------------------------------")


def parse_F02F(regs):
    """
    Bloc 0xF02F-0xF03A (13 registri) — energie zilnica si cumulativa.
    Confirmat functional pe HF2450S80H.
    """
    def r(a):
        i = a - 0xF02F
        return regs[i] if 0 <= i < len(regs) else 0

    p("  +-- Bloc 0xF02F (energie zilnica + cumulativa) -----------")
    p(f"  |  [F02F] PV azi:        {r(0xF02F) * 0.1:.1f} kWh")
    p(f"  |  [F030] Sarcina azi:   {r(0xF030) * 0.1:.1f} kWh")
    p(f"  |  [F031] raw:           {r(0xF031)} (neidentificat)")
    p(f"  |  [F032] raw:           {r(0xF032)} (neidentificat)")
    p(f"  |  [F038] PV total:      {r(0xF038) * 0.1:.1f} kWh")
    p(f"  |  [F03A] Sarcina total: {r(0xF03A) * 0.1:.1f} kWh")
    p("  +----------------------------------------------------------")

# --- Scanare porturi ----------------------------------------------------------

def scan_ports():
    p("")
    p("=== Porturi seriale disponibile ==============================")
    by_id_dir = "/dev/serial/by-id/"
    if os.path.isdir(by_id_dir):
        links = sorted(glob.glob(by_id_dir + "*"))
        if links:
            p("  /dev/serial/by-id/:")
            for link in links:
                p(f"    {os.path.basename(link)}")
                p(f"      -> {os.path.realpath(link)}")
        else:
            p("  /dev/serial/by-id/ gol")
    else:
        p("  /dev/serial/by-id/ indisponibil")
    tty_list = sorted(glob.glob("/dev/ttyUSB*"))
    p(f"  ttyUSB: {', '.join(tty_list) if tty_list else 'niciun'}")
    p("")

# --- Main ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SRNE Modbus RTU Debug Tool v1.4",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemple:
  python3 srne_debug.py /dev/ttyUSB1
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300
  python3 srne_debug.py --scan

Log salvat langa script: srne_debug.log
        """
    )
    parser.add_argument("port",   nargs="?", default="/dev/ttyUSB1")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=1)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--scan", action="store_true")
    parser.add_argument("--reg",  nargs=2, metavar=("START", "COUNT"))
    parser.add_argument("--write",nargs=2, metavar=("REG", "VALUE"))
    parser.add_argument("--raw",  action="store_true")
    args = parser.parse_args()

    p("=" * 60)
    p("  SRNE Invertor - Modbus RTU Debug Tool v1.4")
    p(f"  Log: {LOG_FILE}")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p("=" * 60)

    scan_ports()

    if args.scan and not args.reg and not args.write:
        p(f"Log: {LOG_FILE}")
        return

    p(f"  Port: {args.port} | Baud: {args.baud} 8N1 | Addr: {args.addr}")
    p("")

    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8,
                            parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)
        p(f"  Port deschis OK: {args.port}")
        p("")
    except serial.SerialException as e:
        p(f"  FAIL Nu pot deschide {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.3)

    # --- Citire registru specific ---
    if args.reg:
        reg_start = int(args.reg[0], 0)
        count     = int(args.reg[1])
        p(f"=== Citire 0x{reg_start:04X} x {count} registri ========================")
        regs = read_regs(ser, args.addr, reg_start, count)
        if regs:
            p("")
            p("  Registru    Dec(u)  Dec(s)   Hex     Binar")
            p("  " + "-" * 58)
            for i, v in enumerate(regs):
                s = to_signed16(v)
                p(f"  [0x{reg_start+i:04X}]  {v:>6}  {s:>7}  0x{v:04X}  {v:016b}b")
        ser.close()
        p(f"\nLog: {LOG_FILE}")
        return

    # --- Scriere registru ---
    if args.write:
        reg = int(args.write[0], 0)
        val = int(args.write[1], 0)
        p(f"=== Scriere 0x{reg:04X} = {val} (0x{val:04X}) =========================")
        ok = write_reg(ser, args.addr, reg, val)
        p(f"  Rezultat: {'OK' if ok else 'FAIL'}")
        ser.close()
        p(f"\nLog: {LOG_FILE}")
        return

    # --- Test complet ---
    p("=== Test complet =============================================")
    p("")

    p("Citire 0x0100 x 15 (baterie + string PV)...")
    r0100 = read_regs(ser, args.addr, 0x0100, 15)
    if r0100:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in r0100]}")
        else: parse_0100_15(r0100)
    time.sleep(0.2)

    p("")
    p("Citire 0x0204 x 31 (AC output + RTC + temperaturi)...")
    r0204 = read_regs(ser, args.addr, 0x0204, 31)
    if r0204:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in r0204]}")
        else: parse_0204(r0204)
    time.sleep(0.2)

    p("")
    p("Citire 0xF02F x 13 (energie zilnica + cumulativa)...")
    rF02F = read_regs(ser, args.addr, 0xF02F, 13)
    if rF02F:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in rF02F]}")
        else: parse_F02F(rF02F)
    time.sleep(0.2)

    p("")
    p("Citire E-registri (machine state + fault)...")
    for reg, desc in [(0xE004, "Machine state"), (0xE204, "Fault/alarm")]:
        p(f"  {desc} (0x{reg:04X}):")
        regs = read_regs(ser, args.addr, reg, 1)
        if regs:
            v = regs[0]
            if reg == 0xE004:
                p(f"    {v} -> {MACHINE_STATE.get(v, f'Unknown({v})')}")
            else:
                p(f"    {v} -> {'OK' if v == 0 else f'FAULT 0x{v:04X}'}")
        time.sleep(0.2)

    p("")
    p("=== Test complet finalizat ===================================")
    p(f"Log: {LOG_FILE}")
    ser.close()

if __name__ == "__main__":
    main()
