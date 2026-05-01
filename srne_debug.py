#!/usr/bin/env python3
"""
srne_debug.py v1.3 - Diagnosticare SRNE Invertor Modbus RTU
============================================================
Rulare din terminal SSH pe HA:
  pip install pyserial
  python3 /config/addons/srne_invertor/srne_debug.py /dev/ttyUSB1

Output salvat automat in acelasi folder cu scriptul (srne_debug.log)

Exemple:
  python3 srne_debug.py /dev/ttyUSB1              # test complet
  python3 srne_debug.py --scan                    # scanare porturi
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15  # citire bloc baterie
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1   # citire machine state
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300  # scriere registru
  python3 srne_debug.py /dev/ttyUSB1 --raw        # dump hex brut

Arhitectura bus RS485:
  - Pe bus-ul intern: Invertorul = MASTER (intreaba BMS-ul periodic)
                      BMS JBD   = SLAVE  (raspunde invertorului)
  - Noi ne conectam ca un al doilea master extern:
    Scriptul = MASTER extern, Invertorul = SLAVE la adresa 1
  - Receive-ul filtreaza dupa adresa 0x01 + CRC valid
    Traficul invertor->BMS (0xFF) este ignorat automat

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

# --- Logger dual: consola + fisier langa script -------------------------------

# Log in acelasi folder cu scriptul, indiferent de unde e rulat
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
    """Print + log in fisier."""
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

# --- Receive cu filtrare dupa adresa slave ------------------------------------

def recv_slave_frame(ser, slave_addr, expected_regs, timeout=3.0):
    """
    Primeste raspuns Modbus RTU de la slave_addr, ignorand orice alt trafic.

    Scanam byte cu byte:
    - Daca byte != slave_addr → ignorat (poate fi trafic invertor->BMS)
    - Daca byte == slave_addr → incercam sa construim un frame valid
    - Validam: FC=0x03, byte_count corect, CRC valid

    Raspuns normal FC03:
      [slave_addr][0x03][byte_count][data...][CRC_L][CRC_H]

    Raspuns exception:
      [slave_addr][0x83][exc_code][CRC_L][CRC_H]
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
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16(frame[:-2])
                if crc_recv == crc_calc:
                    if ignored_bytes > 0:
                        p(f"  [bus] ignorat {ignored_bytes}b trafic de la alte dispozitive")
                    return frame, False
                i += 1
                continue

            # Raspuns exception FC03 (0x83)
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16(frame[:-2])
                if crc_recv == crc_calc:
                    if ignored_bytes > 0:
                        p(f"  [bus] ignorat {ignored_bytes}b trafic de la alte dispozitive")
                    return frame, True
                i += 1
                continue

            i += 1

    if ignored_bytes > 0:
        p(f"  [bus] ignorat {ignored_bytes}b trafic de la alte dispozitive (timeout)")
    return None, False


def transact(ser, slave_addr, request, expected_regs, timeout=3.0):
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.write(request)
    p(f"  TX ({len(request)}b): {request.hex(' ').upper()}")

    frame, is_exc = recv_slave_frame(ser, slave_addr, expected_regs, timeout)

    if frame is None:
        p(f"  RX: timeout - niciun raspuns valid de la addr=0x{slave_addr:02X}")
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
    request = build_fc03(slave_addr, reg_start, count)
    frame = transact(ser, slave_addr, request, count)
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

# --- Parsare SRNE -------------------------------------------------------------

CHARGE_STATE  = {0:"Off", 1:"Active", 2:"MPPT", 3:"Equalizing",
                 4:"Boost", 5:"Float", 6:"Current limit"}
MACHINE_STATE = {0:"Standby", 1:"No anomaly", 2:"SW startup", 3:"Starting",
                 4:"Line mode", 5:"Inverter mode", 6:"ECO mode",
                 7:"Fault", 8:"Shutdown", 9:"Running (inverter)"}

def parse_0100_15(regs):
    def r(a):
        i = a - 0x0100
        return regs[i] if 0 <= i < len(regs) else 0
    t  = r(0x0103)
    tc = (t >> 8) & 0x7F
    tb = t & 0x7F
    cs = r(0x010C) & 0xFF
    p("  +-- Bloc 0x0100-0x010E (15 regs: baterie + PV) -----------")
    p(f"  |  SOC:           {r(0x0100) & 0xFF}%")
    p(f"  |  Vbat:          {r(0x0101) * 0.1:.1f} V  (raw: {r(0x0101):#06x})")
    p(f"  |  Ibat:          {r(0x0102) * 0.1:.1f} A  (raw: {r(0x0102):#06x})")
    p(f"  |  Temp ctrl:     {tc}C  | Temp bat: {tb}C")
    p(f"  |  Vpv:           {r(0x0107) * 0.1:.1f} V")
    p(f"  |  Ipv:           {r(0x0108) * 0.01:.2f} A")
    p(f"  |  Ppv:           {r(0x0109)} W")
    p(f"  |  Load on/off:   {r(0x010A)}")
    p(f"  |  Vbat min azi:  {r(0x010B) * 0.1:.1f} V")
    p(f"  |  Charge step:   {CHARGE_STATE.get(cs, f'?({cs})')}  (raw: {cs})")
    p("  +----------------------------------------------------------")

def parse_0113(regs):
    def r(a):
        i = a - 0x0113
        return regs[i] if 0 <= i < len(regs) else 0
    p("  +-- Bloc 0x0113 (energie zilnica + statistici) ------------")
    p(f"  |  PV azi (Wh):       {r(0x0113)}")
    p(f"  |  Consum azi (Wh):   {r(0x0114)}")
    p(f"  |  Zile operare:      {r(0x0115)}")
    p(f"  |  Over-discharge:    {r(0x0116)}")
    p(f"  |  Full charges:      {r(0x0117)}")
    p("  +----------------------------------------------------------")

def parse_0121(regs):
    fault = (regs[0] << 16) | regs[1]
    p("  +-- Fault 0x0121-0x0122 -----------------------------------")
    p(f"  |  Fault word:    0x{fault:08X}  {'OK' if fault == 0 else 'FAULT!'}")
    if fault:
        bits = {0:"Bat over-discharge", 1:"Bat over-voltage", 2:"Bat under-voltage",
                3:"Load short-circuit", 4:"Load overpower", 5:"Ctrl temp high",
                7:"PV overpower", 9:"PV over-voltage", 12:"PV reverse"}
        for bit, desc in bits.items():
            if fault & (1 << bit):
                p(f"  |  [B{bit:02d}] {desc}")
    p("  +----------------------------------------------------------")

def parse_0204(regs):
    def r(a):
        i = a - 0x0204
        return regs[i] if 0 <= i < len(regs) else 0
    ms  = r(0x0209) & 0xFF
    r0, r1, r2 = r(0x020C), r(0x020D), r(0x020E)
    pac = r(0x021B)
    pap = r(0x021C)
    p("  +-- Bloc 0x0204 (AC output + temps + RTC) -----------------")
    p(f"  |  Machine state: {MACHINE_STATE.get(ms, f'?({ms})')}  (raw: {ms})")
    p(f"  |  RTC:           {(r0>>8)+2002}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}")
    p(f"  |  Load ratio:    {r(0x0210)}%")
    p(f"  |  Vac out:       {r(0x0216) * 0.1:.1f} V")
    p(f"  |  Fac out:       {r(0x0218) * 0.01:.2f} Hz")
    p(f"  |  Iac out:       {r(0x0219) * 0.1:.1f} A")
    p(f"  |  Pac active:    {pac} W")
    p(f"  |  Pac apparent:  {pap} VA")
    p(f"  |  Power factor:  {round(pac/pap,3) if pap else 0.0:.3f}")
    p(f"  |  Temp DC side:  {r(0x0220) * 0.1:.1f}C")
    p(f"  |  Temp AC side:  {r(0x0221) * 0.1:.1f}C")
    p(f"  |  Temp trafo:    {r(0x0222) * 0.1:.1f}C")
    p("  +----------------------------------------------------------")

def parse_F02F(regs):
    def r(a):
        i = a - 0xF02F
        return regs[i] if 0 <= i < len(regs) else 0
    p("  +-- Bloc 0xF02F (energie totala) --------------------------")
    p(f"  |  PV azi:        {r(0xF02F) * 0.1:.1f} kWh")
    p(f"  |  Sarcina azi:   {r(0xF030) * 0.1:.1f} kWh")
    p(f"  |  [F031] raw:    {r(0xF031)}")
    p(f"  |  [F032] raw:    {r(0xF032)}")
    p(f"  |  PV total:      {r(0xF038) * 0.1:.1f} kWh")
    p(f"  |  Sarcina total: {r(0xF03A) * 0.1:.1f} kWh")
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
        description="SRNE Modbus RTU Debug Tool v1.3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Exemple:
  python3 srne_debug.py /dev/ttyUSB1
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300
  python3 srne_debug.py --scan

Log salvat automat langa script (srne_debug.log)
        """
    )
    parser.add_argument("port",   nargs="?", default="/dev/ttyUSB1")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=1,
                        help="Adresa Modbus slave (default: 1)")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--scan", action="store_true")
    parser.add_argument("--reg",  nargs=2, metavar=("START", "COUNT"))
    parser.add_argument("--write",nargs=2, metavar=("REG", "VALUE"))
    parser.add_argument("--raw",  action="store_true")
    args = parser.parse_args()

    p("=" * 60)
    p("  SRNE Invertor - Modbus RTU Debug Tool v1.3")
    p(f"  Log: {LOG_FILE}")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p("=" * 60)
    p("  Arhitectura bus:")
    p("    Intern: Invertor(master) <-> BMS(slave) — trafic propriu")
    p("    Extern: Script(master)   -> Invertor(slave addr=1)")
    p("    Receive filtreaza dupa addr=0x01 + CRC valid")

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

    if args.reg:
        reg_start = int(args.reg[0], 0)
        count     = int(args.reg[1])
        p(f"=== Citire 0x{reg_start:04X} x {count} registri ========================")
        regs = read_regs(ser, args.addr, reg_start, count)
        if regs:
            p("")
            p("  Registru    Dec     Hex     Binar")
            p("  " + "-" * 50)
            for i, v in enumerate(regs):
                p(f"  [0x{reg_start+i:04X}]  {v:>6}  0x{v:04X}  {v:016b}b")
        ser.close()
        p(f"\nLog: {LOG_FILE}")
        return

    if args.write:
        reg = int(args.write[0], 0)
        val = int(args.write[1], 0)
        p(f"=== Scriere 0x{reg:04X} = {val} (0x{val:04X}) =========================")
        ok = write_reg(ser, args.addr, reg, val)
        p(f"  Rezultat: {'OK' if ok else 'FAIL'}")
        ser.close()
        p(f"\nLog: {LOG_FILE}")
        return

    p("=== Test complet =============================================")
    p("")

    p("Citire 0x0100 x 15 (baterie + PV)...")
    r0100 = read_regs(ser, args.addr, 0x0100, 15)
    if r0100:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in r0100]}")
        else: parse_0100_15(r0100)
    time.sleep(0.2)

    p("")
    p("Citire 0x0113 x 5 (energie zilnica + statistici)...")
    r0113 = read_regs(ser, args.addr, 0x0113, 5)
    if r0113:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in r0113]}")
        else: parse_0113(r0113)
    time.sleep(0.2)

    p("")
    p("Citire 0x0121 x 2 (fault word)...")
    r0121 = read_regs(ser, args.addr, 0x0121, 2)
    if r0121:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in r0121]}")
        else: parse_0121(r0121)
    time.sleep(0.2)

    p("")
    p("Citire 0x0204 x 31 (AC output + temps + RTC)...")
    r0204 = read_regs(ser, args.addr, 0x0204, 31)
    if r0204:
        if args.raw: p(f"  Raw: {[f'0x{v:04X}' for v in r0204]}")
        else: parse_0204(r0204)
    time.sleep(0.2)

    p("")
    p("Citire 0xF02F x 13 (energie totala)...")
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
