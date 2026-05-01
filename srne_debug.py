#!/usr/bin/env python3
"""
srne_debug.py v1.1 - Diagnosticare SRNE Invertor Modbus RTU
============================================================
Rulare din terminal SSH pe HA:
  pip install pyserial
  python3 srne_debug.py /dev/ttyUSB1

Exemple:
  python3 srne_debug.py /dev/ttyUSB1              # test complet
  python3 srne_debug.py --scan                    # scanare porturi
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15  # citire bloc baterie
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1   # citire machine state
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300  # scriere registru
  python3 srne_debug.py /dev/ttyUSB1 --raw        # dump hex brut
  python3 srne_debug.py /dev/ttyUSB1 --baud 9600  # viteza alternativa

Nota bus-sharing: CH341 de la invertor si JBD BMS sunt pe acelasi bus RS485
fizic. Receive-ul filtreaza automat traficul JBD (adresa 0xFF) si cauta
raspunsul valid al invertorului (adresa 0x01).

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

# --- Receive inteligent cu filtrare zgomot bus --------------------------------

def recv_modbus_frame(ser, addr, expected_regs, timeout=3.0):
    """
    Citeste bytes de pe bus si cauta un frame Modbus valid pentru adresa addr.
    Ignora traficul de la alte dispozitive (ex. JBD BMS cu adresa 0xFF).

    Returneaza (frame_bytes, is_exception) sau (None, False) la timeout.
    """
    buf = bytearray()
    start = time.time()

    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

        # Scanam buffer-ul cautand inceputul unui frame valid
        i = 0
        while i < len(buf):
            # Sarim bytes care nu sunt adresa noastra
            if buf[i] != addr:
                i += 1
                continue

            # Avem adresa corecta - verificam daca avem suficienti bytes
            rest = buf[i:]

            # Frame normal FC03: addr(1) + 0x03(1) + byte_count(1) + data + crc(2)
            if len(rest) >= 3 and rest[1] == 0x03:
                byte_count = rest[2]
                total = 3 + byte_count + 2
                if byte_count != expected_regs * 2:
                    i += 1
                    continue
                if len(rest) < total:
                    break  # asteptam mai multi bytes
                frame = bytes(rest[:total])
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16(frame[:-2])
                if crc_recv == crc_calc:
                    return frame, False
                else:
                    i += 1
                    continue

            # Frame exception: addr(1) + (0x03|0x80)(1) + exc_code(1) + crc(2)
            if len(rest) >= 5 and rest[1] == (0x03 | 0x80):
                frame = bytes(rest[:5])
                crc_recv = struct.unpack("<H", frame[-2:])[0]
                crc_calc = crc16(frame[:-2])
                if crc_recv == crc_calc:
                    return frame, True
                else:
                    i += 1
                    continue

            i += 1

    return None, False


def transact(ser, addr, request, expected_regs, timeout=3.0):
    """Trimite cerere FC03 si asteapta raspuns valid, filtrand zgomotul de pe bus."""
    ser.reset_input_buffer()
    time.sleep(0.05)  # pauza pentru a lasa bus-ul sa se limpezeasca
    ser.write(request)
    print(f"  TX ({len(request)}b): {request.hex(' ').upper()}")

    frame, is_exc = recv_modbus_frame(ser, addr, expected_regs, timeout)

    if frame is None:
        print(f"  RX: timeout sau niciun frame valid gasit")
        return None

    print(f"  RX ({len(frame)}b): {frame.hex(' ').upper()}")

    if is_exc:
        exc_code = frame[2]
        exc_map = {1:"FC nesuportat", 2:"Adresa invalida", 3:"Count prea mare",
                   4:"Eroare citire", 10:"Limita bloc depasita"}
        print(f"  FAIL Modbus exception 0x{exc_code:02X}: {exc_map.get(exc_code, f'cod {exc_code}')}")
        return None

    print(f"  OK")
    return frame


def read_regs(ser, addr, reg_start, count):
    """Citeste count registri de la reg_start. Returneaza lista uint16 sau None."""
    request = build_fc03(addr, reg_start, count)
    frame = transact(ser, addr, request, count)
    if frame is None:
        return None
    return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(count)]


def write_reg(ser, addr, reg, value):
    """Scrie un singur registru (FC06). Returneaza True la succes."""
    request = build_fc06(addr, reg, value)
    ser.reset_input_buffer()
    ser.write(request)
    print(f"  TX ({len(request)}b): {request.hex(' ').upper()}")
    # Raspunsul FC06 = echo al cererii (8 bytes)
    resp = bytearray()
    start = time.time()
    while len(resp) < 8 and time.time() - start < 2.0:
        chunk = ser.read(8 - len(resp))
        if chunk:
            resp.extend(chunk)
    if len(resp) == 8:
        print(f"  RX ({len(resp)}b): {resp.hex(' ').upper()}")
        crc_ok = struct.unpack("<H", bytes(resp[-2:]))[0] == crc16(bytes(resp[:-2]))
        print(f"  {'OK' if crc_ok else 'FAIL CRC'}")
        return crc_ok
    print(f"  FAIL raspuns scurt ({len(resp)}b)")
    return False

# --- Parsare SRNE -------------------------------------------------------------

CHARGE_STATE  = {0:"Off", 1:"Active", 2:"MPPT", 3:"Equalizing",
                 4:"Boost", 5:"Float", 6:"Current limit"}
MACHINE_STATE = {0:"Standby", 1:"No anomaly", 2:"SW startup", 3:"Starting",
                 4:"Line mode", 5:"Inverter mode", 6:"ECO mode",
                 7:"Fault", 8:"Shutdown", 9:"Running (inverter)"}

def parse_0100_15(regs):
    """Bloc 0x0100, primii 15 registri (0x0100-0x010E) - ca iPower.net"""
    def r(a):
        i = a - 0x0100
        return regs[i] if 0 <= i < len(regs) else 0
    t = r(0x0103)
    tc = (t >> 8) & 0x7F
    tb = t & 0x7F
    cs = r(0x010C) & 0xFF
    print("  +-- Bloc 0x0100-0x010E (15 regs) --------------------------")
    print(f"  |  SOC:           {r(0x0100) & 0xFF}%")
    print(f"  |  Vbat:          {r(0x0101) * 0.1:.1f} V  (raw: {r(0x0101):#06x})")
    print(f"  |  Ibat:          {r(0x0102) * 0.1:.1f} A  (raw: {r(0x0102):#06x})")
    print(f"  |  Temp ctrl:     {tc}C  | Temp bat: {tb}C")
    print(f"  |  Vpv:           {r(0x0107) * 0.1:.1f} V")
    print(f"  |  Ipv:           {r(0x0108) * 0.01:.2f} A")
    print(f"  |  Ppv:           {r(0x0109)} W")
    print(f"  |  Load on/off:   {r(0x010A)}")
    print(f"  |  Vbat min azi:  {r(0x010B) * 0.1:.1f} V")
    print(f"  |  Charge step:   {CHARGE_STATE.get(cs, f'?({cs})')}  (raw: {cs})")
    print(f"  +----------------------------------------------------------")

def parse_0113_10(regs, base=0x0113):
    """Bloc 0x0113-0x011C (energie zilnica + istorica)"""
    def r(a):
        i = a - base
        return regs[i] if 0 <= i < len(regs) else 0
    print(f"  +-- Bloc 0x{base:04X} (energie zilnica + istorica) -----------")
    print(f"  |  PV azi (Wh):       {r(0x0113)}")
    print(f"  |  Consum azi (Wh):   {r(0x0114)}")
    print(f"  |  Zile operare:      {r(0x0115)}")
    print(f"  |  Over-discharge:    {r(0x0116)}")
    print(f"  |  Full charges:      {r(0x0117)}")
    print(f"  +----------------------------------------------------------")

def parse_0121_2(regs):
    """Registri fault 0x0121-0x0122"""
    fault = (regs[0] << 16) | regs[1]
    print(f"  +-- Fault 0x0121-0x0122 -----------------------------------")
    print(f"  |  Fault word:    0x{fault:08X}  {'OK' if fault == 0 else 'FAULT!'}")
    if fault:
        bits = {0:"Bat over-discharge", 1:"Bat over-voltage", 2:"Bat under-voltage",
                3:"Load short-circuit", 4:"Load overpower", 5:"Ctrl temp high",
                7:"PV overpower", 9:"PV over-voltage", 12:"PV reverse"}
        for bit, desc in bits.items():
            if fault & (1 << bit):
                print(f"  |  [B{bit:02d}] {desc}")
    print(f"  +----------------------------------------------------------")

def parse_0204(regs):
    def r(a):
        i = a - 0x0204
        return regs[i] if 0 <= i < len(regs) else 0
    ms  = r(0x0209) & 0xFF
    r0, r1, r2 = r(0x020C), r(0x020D), r(0x020E)
    pac = r(0x021B)
    pap = r(0x021C)
    print(f"  +-- Bloc 0x0204 - Iesire AC + Temps -----------------------")
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
    print(f"  +-- Bloc 0xF02F - Energie ---------------------------------")
    print(f"  |  PV azi:        {r(0xF02F) * 0.1:.1f} kWh")
    print(f"  |  Sarcina azi:   {r(0xF030) * 0.1:.1f} kWh")
    print(f"  |  [F031]:        {r(0xF031)} (raw)")
    print(f"  |  [F032]:        {r(0xF032)} (raw)")
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
        print("  /dev/serial/by-id/ indisponibil")
    tty_list = sorted(glob.glob("/dev/ttyUSB*"))
    print(f"\n  ttyUSB: {', '.join(tty_list) if tty_list else 'niciun'}")
    print()

# --- Main ---------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SRNE Modbus RTU Debug Tool v1.1",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemple:
  python3 srne_debug.py /dev/ttyUSB1
  python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15
  python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1
  python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300
  python3 srne_debug.py --scan
        """
    )
    parser.add_argument("port",  nargs="?", default="/dev/ttyUSB1")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=1)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--scan", action="store_true")
    parser.add_argument("--reg",  nargs=2, metavar=("START", "COUNT"))
    parser.add_argument("--write",nargs=2, metavar=("REG", "VALUE"))
    parser.add_argument("--raw",  action="store_true")
    args = parser.parse_args()

    print("=" * 60)
    print("  SRNE Invertor - Modbus RTU Debug Tool v1.1")
    print("=" * 60)

    scan_ports()

    if args.scan and not args.reg and not args.write:
        return

    print(f"  Port: {args.port} | Baud: {args.baud} 8N1 | Addr: {args.addr}")
    print(f"  Nota: receive filtreaza automat zgomotul de la alte dispozitive pe bus\n")

    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8,
                            parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)
        print(f"  Port deschis OK: {args.port}\n")
    except serial.SerialException as e:
        print(f"  FAIL Nu pot deschide {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.3)

    # --- Citire registru specific ---
    if args.reg:
        reg_start = int(args.reg[0], 0)
        count     = int(args.reg[1])
        print(f"=== Citire 0x{reg_start:04X} x {count} registri ========================")
        regs = read_regs(ser, args.addr, reg_start, count)
        if regs:
            print("\n  Valori (decimal | hex | binar):")
            for i, v in enumerate(regs):
                print(f"    [0x{reg_start+i:04X}]  {v:>6}  0x{v:04X}  {v:016b}b")
        ser.close()
        return

    # --- Scriere registru ---
    if args.write:
        reg = int(args.write[0], 0)
        val = int(args.write[1], 0)
        print(f"=== Scriere 0x{reg:04X} = {val} (0x{val:04X}) =========================")
        ok = write_reg(ser, args.addr, reg, val)
        print(f"\n  Rezultat: {'OK' if ok else 'FAIL'}")
        ser.close()
        return

    # --- Test complet ---
    print("=== Test complet =============================================\n")

    # Bloc 0x0100: 15 regs (ca iPower.net - evita exception la 35 regs)
    print("Citire 0x0100 x 15 (baterie + PV - format iPower.net)...")
    r0100 = read_regs(ser, args.addr, 0x0100, 15)
    if r0100:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in r0100])
        else: parse_0100_15(r0100)
    time.sleep(0.2)

    # Bloc 0x0113: energie zilnica + istorica
    print("\nCitire 0x0113 x 5 (energie zilnica + statistici)...")
    r0113 = read_regs(ser, args.addr, 0x0113, 5)
    if r0113:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in r0113])
        else: parse_0113_10(r0113, base=0x0113)
    time.sleep(0.2)

    # Registri fault
    print("\nCitire 0x0121 x 2 (fault word)...")
    r0121 = read_regs(ser, args.addr, 0x0121, 2)
    if r0121:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in r0121])
        else: parse_0121_2(r0121)
    time.sleep(0.2)

    # Bloc 0x0204: AC output + temps + RTC
    print("\nCitire 0x0204 x 31 (AC output + temps + RTC)...")
    r0204 = read_regs(ser, args.addr, 0x0204, 31)
    if r0204:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in r0204])
        else: parse_0204(r0204)
    time.sleep(0.2)

    # Bloc 0xF02F: energie totala
    print("\nCitire 0xF02F x 13 (energie totala)...")
    rF02F = read_regs(ser, args.addr, 0xF02F, 13)
    if rF02F:
        if args.raw: print("  Raw:", [f"0x{v:04X}" for v in rF02F])
        else: parse_F02F(rF02F)
    time.sleep(0.2)

    # E-registri
    print("\nCitire E-registri (machine state + fault)...")
    for reg, desc in [(0xE004, "Machine state"), (0xE204, "Fault/alarm")]:
        print(f"\n  {desc} (0x{reg:04X}):")
        regs = read_regs(ser, args.addr, reg, 1)
        if regs:
            v = regs[0]
            if reg == 0xE004:
                print(f"    {v} -> {MACHINE_STATE.get(v, f'Unknown({v})')}")
            else:
                print(f"    {v} -> {'OK' if v == 0 else f'FAULT 0x{v:04X}'}")
        time.sleep(0.2)

    print("\n=== Test complet finalizat ===================================\n")
    ser.close()

if __name__ == "__main__":
    main()
