#!/usr/bin/env python3
"""
srne_scan.py - Scanare completa registri SRNE Energy Storage Inverter
=====================================================================
Conform: MODBUS Protocol for Energy Storage Inverter v1.96 (2024)
         PythonProtocolGateway srne_2021_v1.96.holding_registry_map.csv

Testeaza toate blocurile de registri definite in protocol si raporteaza
ce exista si ce returneaza exception pe hardware-ul conectat.

Rulare:
  pip install pyserial
  python3 srne_scan.py /dev/ttyUSB1
  python3 srne_scan.py /dev/ttyUSB1 --area all
  python3 srne_scan.py /dev/ttyUSB1 --area product
  python3 srne_scan.py /dev/ttyUSB1 --area live
  python3 srne_scan.py /dev/ttyUSB1 --area stats
  python3 srne_scan.py /dev/ttyUSB1 --area settings

Log salvat: srne_scan.log (langa script)

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import sys
import time
import struct
import argparse
import os
import logging
from datetime import datetime

try:
    import serial
except ImportError:
    print("Lipsa: pip install pyserial")
    sys.exit(1)

# --- Logger -------------------------------------------------------------------

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "srne_scan.log")

def setup_logger():
    logger = logging.getLogger("srne_scan")
    logger.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S")
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    logger.addHandler(ch)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt)
    logger.addHandler(fh)
    return logger

log = logging.getLogger("srne_scan")
def p(msg=""): log.info(msg)

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

# --- Receive cu filtrare adresa slave -----------------------------------------

def recv_slave_frame(ser, slave_addr, expected_regs, timeout=3.0):
    buf = bytearray()
    start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        i = 0
        while i < len(buf):
            if buf[i] != slave_addr:
                i += 1
                continue
            rest = buf[i:]
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
                    return frame, False
                i += 1
                continue
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == crc16(frame[:-2]):
                    return frame, True
                i += 1
                continue
            i += 1
    return None, False

def read_regs(ser, addr, reg_start, count, timeout=3.0):
    request = build_fc03(addr, reg_start, count)
    ser.reset_input_buffer()
    time.sleep(0.06)
    ser.write(request)
    frame, is_exc = recv_slave_frame(ser, addr, count, timeout)
    if frame is None:
        return None, 'timeout'
    if is_exc:
        return None, f'exception_0x{frame[2]:02X}'
    regs = [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(count)]
    return regs, 'ok'

# --- Helpers ------------------------------------------------------------------

def to_signed16(v): return v if v < 0x8000 else v - 0x10000
def u32(lo, hi): return (hi << 16) | lo

def decode_string_regs(regs):
    chars = []
    for r in regs:
        lo = r & 0xFF
        if lo == 0:
            break
        chars.append(chr(lo) if 32 <= lo < 127 else '?')
    return ''.join(chars).strip()

# --- Enums --------------------------------------------------------------------

BATTERY_TYPE = {
    0:"User define", 1:"SLD", 2:"FLD", 3:"GEL",
    4:"LiFePO4 x14", 5:"LiFePO4 x15", 6:"LiFePO4 x16",
    7:"LiFePO4 x7",  8:"LiFePO4 x8",  9:"LiFePO4 x9",
    10:"Ternary Li x7", 11:"Ternary Li x8",
    12:"Ternary Li x13", 13:"Ternary Li x14",
}
CHARGE_STATE = {
    0:"Off", 1:"Quick charge", 2:"Const voltage",
    4:"Float", 6:"Li activate", 8:"Full",
}
MACHINE_STATE = {
    0:"Init", 1:"Standby", 2:"AC power operation", 3:"Inverter operation",
    4:"AC power", 5:"Inverter", 6:"Inv->AC", 7:"AC->Inv",
    8:"Bat activate", 9:"Manual shutdown", 10:"Fault",
}
OUTPUT_PRIORITY = {
    0:"Solar first", 1:"Grid first", 2:"SBU", 3:"Solar only / Off-grid",
}
PRODUCT_TYPE = {
    0:"Domestic controller", 1:"Street light controller",
    3:"Grid-connected inverter", 4:"All-in-one solar charger inverter",
    5:"Power frequency off-grid",
}
PARALLEL_MODE = {
    0:"Single", 1:"Single-phase parallel", 2:"Two-phase parallel",
    3:"Two-phase 120", 4:"Two-phase 180",
    5:"Three-phase A", 6:"Three-phase B", 7:"Three-phase C",
}

# --- Scan functions -----------------------------------------------------------

def scan_product_info(ser, addr):
    p()
    p("=" * 65)
    p("  P00 - PRODUCT INFORMATION AREA (0x000A - 0x004A)")
    p("=" * 65)

    regs, status = read_regs(ser, addr, 0x000A, 6)
    if status == 'ok':
        p(f"  [000A] MinorVersion:       {regs[0]}")
        pt = regs[1]
        p(f"  [000B] ProductType:        {pt} = {PRODUCT_TYPE.get(pt, f'?({pt})')}")
    else:
        p(f"  [000A-000F] -> {status}")

    regs, status = read_regs(ser, addr, 0x0014, 4)
    if status == 'ok':
        p(f"  [0014] APP version:        {regs[0]} = V{regs[0]/100:.2f}")
        p(f"  [0015] BOOT version:       {regs[1]} = V{regs[1]/100:.2f}")
        p(f"  [0016] HW ctrl version:    {regs[2]} = V{regs[2]/100:.2f}")
        p(f"  [0017] HW power version:   {regs[3]} = V{regs[3]/100:.2f}")
    else:
        p(f"  [0014-0017] -> {status}")

    regs, status = read_regs(ser, addr, 0x001A, 8)
    if status == 'ok':
        p(f"  [001A] RS485 Address:      {regs[0]}")
        p(f"  [001B] Model code:         {regs[1]}")
        p(f"  [001C] Protocol version:   {regs[2]} = V{regs[2]/100:.2f}")
        yr = (regs[4] >> 8) + 2000
        p(f"  [001E] Manufacture date:   {yr}-{regs[4]&0xFF:02d}-{regs[5]>>8:02d} {regs[5]&0xFF:02d}h")
        area = {0:"Shenzhen", 1:"Dongguan"}.get(regs[6], f"?({regs[6]})")
        p(f"  [0020] Product area:       {area}")
    else:
        p(f"  [001A-0020] -> {status}")

    regs, status = read_regs(ser, addr, 0x0021, 20)
    if status == 'ok':
        p(f"  [0021-0034] CPU build:     '{decode_string_regs(regs)}'")
    else:
        p(f"  [0021-0034] -> {status}")

    regs, status = read_regs(ser, addr, 0x0035, 20)
    if status == 'ok':
        p(f"  [0035-0048] Serial No:     '{decode_string_regs(regs)}'")
    else:
        p(f"  [0035-0048] -> {status}")


def scan_live_dc(ser, addr):
    p()
    p("=" * 65)
    p("  P01 - LIVE DC DATA (0x0100 - 0x0111)")
    p("=" * 65)

    regs, status = read_regs(ser, addr, 0x0100, 18)
    if status == 'ok':
        ibat = to_signed16(regs[2])
        tbat = to_signed16(regs[3])
        cs   = regs[11]
        p(f"  [0100] SOC:                {regs[0] & 0xFF}%")
        p(f"  [0101] Vbat:               {regs[1] * 0.1:.1f} V")
        p(f"  [0102] Ibat (signed):      {ibat * 0.1:.1f} A  "
          f"({'discharge' if ibat>0 else 'charge' if ibat<0 else 'idle'})")
        p(f"  [0103] Tbat:               {tbat * 0.1:.1f} C  (raw {regs[3]:#06x})")
        p(f"  [0104] reserved:           {regs[4]}")
        p(f"  [0105] reserved:           {regs[5]}")
        p(f"  [0106] reserved:           {regs[6]}")
        p(f"  [0107] Vpv1:               {regs[7] * 0.1:.1f} V")
        p(f"  [0108] Ipv1:               {regs[8] * 0.1:.1f} A")
        p(f"  [0109] Ppv1:               {regs[9]} W")
        p(f"  [010A] PvTotalPower:        {regs[10]} W")
        p(f"  [010B] ChargeState:         {cs} = {CHARGE_STATE.get(cs, f'?({cs})')}")
        p(f"  [010C] reserved:           {regs[12]}")
        p(f"  [010D] reserved:           {regs[13]}")
        p(f"  [010E] TotalChgPower:       {regs[14]} W")
        p(f"  [010F] Vpv2:               {regs[15] * 0.1:.1f} V")
        p(f"  [0110] Ipv2:               {regs[16] * 0.1:.1f} A")
        p(f"  [0111] Ppv2:               {regs[17]} W")
    else:
        p(f"  [0100-0111] -> {status}")


def scan_live_ac(ser, addr):
    p()
    p("=" * 65)
    p("  P02 - LIVE AC DATA (0x0210 - 0x022F)")
    p("=" * 65)

    regs, status = read_regs(ser, addr, 0x0210, 32)
    if status == 'ok':
        ms = regs[0]
        p(f"  [0210] MachineState:       {ms} = {MACHINE_STATE.get(ms, f'?({ms})')}")
        p(f"  [0211] PriorityFlag:       {regs[1]}")
        p(f"  [0212] BusVoltSum:         {regs[2] * 0.1:.1f} V")
        p(f"  [0213] GridVoltA:          {regs[3] * 0.1:.1f} V")
        p(f"  [0214] GridCurrA:          {regs[4] * 0.1:.1f} A")
        p(f"  [0215] GridFreq:           {regs[5] * 0.01:.2f} Hz")
        p(f"  [0216] InvVoltA:           {regs[6] * 0.1:.1f} V")
        p(f"  [0217] InvCurrA:           {regs[7] * 0.1:.1f} A")
        p(f"  [0218] InvFreq:            {regs[8] * 0.01:.2f} Hz")
        p(f"  [0219] LoadCurrA:          {regs[9] * 0.1:.1f} A")
        p(f"  [021A] LoadPF:             {to_signed16(regs[10]) * 0.01:.2f}")
        p(f"  [021B] LoadActivePowerA:   {regs[11]} W")
        p(f"  [021C] LoadApparentPowerA: {regs[12]} VA")
        p(f"  [021D] InvDcVolt:          {to_signed16(regs[13])} mV")
        p(f"  [021E] LineChgCurr:        {regs[14] * 0.1:.1f} A  (AC->bat)")
        p(f"  [021F] LoadRatioA:         {regs[15]}%")
        p(f"  [0220] Temp DC-DC:         {to_signed16(regs[16]) * 0.1:.1f} C")
        p(f"  [0221] Temp DC-AC:         {to_signed16(regs[17]) * 0.1:.1f} C")
        p(f"  [0222] Temp Trafo:         {to_signed16(regs[18]) * 0.1:.1f} C")
        p(f"  [0223] Temp Ambient:       {to_signed16(regs[19]) * 0.1:.1f} C")
        p(f"  [0224] PV->bat chg curr:   {regs[20] * 0.1:.1f} A")
        p(f"  [0225] ParallCurrRms:      {regs[21] * 0.1:.1f} A")
        p(f"  [0226] InvFaultState:      0x{regs[22]:04X}")
        p(f"  [0227] ChargeStatus:       {regs[23]}")
        p(f"  [0228] PBusVolt:           {regs[24] * 0.1:.1f} V")
        p(f"  [0229] NBusVolt:           {regs[25] * 0.1:.1f} V")
        p(f"  [022A] GridVoltB:          {regs[26] * 0.1:.1f} V")
        p(f"  [022B] GridVoltC:          {regs[27] * 0.1:.1f} V")
        p(f"  [022C] InvVoltB:           {regs[28] * 0.1:.1f} V")
        p(f"  [022D] InvVoltC:           {regs[29] * 0.1:.1f} V")
        p(f"  [022E] InvCurrB:           {regs[30] * 0.1:.1f} A")
        p(f"  [022F] InvCurrC:           {regs[31] * 0.1:.1f} A")
    else:
        p(f"  [0210-022F] -> {status}")


def scan_live_rtc(ser, addr):
    p()
    p("  RTC (0x020C-0x020E - older firmware variant):")
    regs, status = read_regs(ser, addr, 0x020C, 3)
    if status == 'ok':
        r0, r1, r2 = regs
        try:
            p(f"  [020C-020E] RTC: {(r0>>8)+2002}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}")
        except Exception as e:
            p(f"  [020C-020E] RTC raw: {regs} ({e})")
    else:
        p(f"  [020C-020E] -> {status}")


def scan_energy_storage_live(ser, addr):
    p()
    p("=" * 65)
    p("  ENERGY STORAGE LIVE DATA (0x7100 - 0x710F)")
    p("=" * 65)
    regs, status = read_regs(ser, addr, 0x7100, 16)
    if status == 'ok':
        for i, v in enumerate(regs):
            p(f"  [71{i:02X}]  {v:6d}  0x{v:04X}  {v:016b}b")
    else:
        p(f"  [7100-710F] -> {status}")


def scan_stats_today(ser, addr):
    p()
    p("=" * 65)
    p("  STATS TODAY + CUMULATIVE (0xF02C - 0xF04D)")
    p("=" * 65)

    regs, status = read_regs(ser, addr, 0xF02C, 8)
    if status == 'ok':
        p(f"  [F02C] PV->Grid azi:       {regs[0] * 0.1:.1f} kWh")
        p(f"  [F02D] Bat chg azi:        {regs[1]} Ah  *** BAT CHG TODAY ***")
        p(f"  [F02E] Bat dischg azi:     {regs[2]} Ah  *** BAT DCHG TODAY ***")
        p(f"  [F02F] PV azi:             {regs[3] * 0.1:.1f} kWh  *** PV TODAY ***")
        p(f"  [F030] Consum azi:         {regs[4] * 0.1:.1f} kWh  *** LOAD TODAY ***")
        p(f"  [F031] Zile operare:       {regs[5]} zile")
        p(f"  [F032-F033] Grid->grid tot:{u32(regs[6], regs[7]) * 0.1:.1f} kWh (32-bit)")
    else:
        p(f"  [F02C-F033] -> {status}")

    time.sleep(0.1)
    regs, status = read_regs(ser, addr, 0xF034, 8)
    if status == 'ok':
        p(f"  [F034-F035] Bat chg TOTAL:  {u32(regs[0], regs[1])} Ah  *** TOTAL CHG AH (DessMonitor) ***")
        p(f"  [F036-F037] Bat dchg TOTAL: {u32(regs[2], regs[3])} Ah  *** TOTAL DCHG AH ***")
        p(f"  [F038-F039] PV TOTAL:       {u32(regs[4], regs[5]) * 0.1:.1f} kWh  *** PV CUMULATIVE ***")
        p(f"  [F03A-F03B] Load TOTAL:     {u32(regs[6], regs[7]) * 0.1:.1f} kWh  *** LOAD CUMULATIVE ***")
    else:
        p(f"  [F034-F03B] -> {status}")

    time.sleep(0.1)
    regs, status = read_regs(ser, addr, 0xF03C, 18)
    if status == 'ok':
        p(f"  [F03C] Grid chg azi:       {regs[0]} Ah")
        p(f"  [F03D] Grid load azi:      {regs[1] * 0.1:.1f} kWh")
        p(f"  [F03E] Inv work azi:       {regs[2]} min")
        p(f"  [F03F] Grid work azi:      {regs[3]} min")
        p(f"  [F040-F042] PowerOnTime:   raw {regs[4]:#06x} {regs[5]:#06x} {regs[6]:#06x}")
        p(f"  [F043-F045] LastEquaChg:   raw {regs[7]:#06x} {regs[8]:#06x} {regs[9]:#06x}")
        p(f"  [F046-F047] Grid chg tot:  {u32(regs[10], regs[11])} Ah")
        p(f"  [F048-F049] Load/grid tot: {u32(regs[12], regs[13]) * 0.1:.1f} kWh")
        p(f"  [F04A] Inv work total:     {regs[14]} h")
        p(f"  [F04B] Grid work total:    {regs[15]} h")
        p(f"  [F04C] Grid chg kWh azi:   {regs[16]}")
        p(f"  [F04D] reserved:           {regs[17]}")
    else:
        p(f"  [F03C-F04D] -> {status}")


def scan_stats_history(ser, addr):
    p()
    p("=" * 65)
    p("  HISTORICAL STATS - LAST 7 DAYS (0xF000 - 0xF01B)")
    p("=" * 65)
    regs, status = read_regs(ser, addr, 0xF000, 28)
    if status == 'ok':
        labels = ["yesterday", "2 days ago", "3 days ago", "4 days ago",
                  "5 days ago", "6 days ago", "7 days ago"]
        p("  PV energy (x0.1 kWh):")
        for i in range(7):
            p(f"    [F{i:03X}] {labels[i]:12s}: {regs[i]:3d} = {regs[i]*0.1:.1f} kWh")
        p("  Bat chg (Ah):")
        for i in range(7):
            p(f"    [F{7+i:03X}] {labels[i]:12s}: {regs[7+i]} Ah")
        p("  Bat dischg (Ah):")
        for i in range(7):
            p(f"    [F{14+i:03X}] {labels[i]:12s}: {regs[14+i]} Ah")
        p("  Grid chg (Ah):")
        for i in range(7):
            p(f"    [F{21+i:03X}] {labels[i]:12s}: {regs[21+i]} Ah")
    else:
        p(f"  [F000-F01B] -> {status}")


def scan_battery_settings(ser, addr):
    p()
    p("=" * 65)
    p("  P05 - BATTERY SETTINGS (0xE000 - 0xE015)")
    p("=" * 65)

    regs, status = read_regs(ser, addr, 0xE000, 22)
    if status == 'ok':
        bat_v    = regs[3]
        bat_type = regs[4]
        v_factor = bat_v / 12.0 if bat_v > 0 else 1.0
        p(f"  [E000] Reserved:            {regs[0]}")
        p(f"  [E001] PV max chg curr:     {regs[1]} A")
        p(f"  [E002] Bat nominal cap:     {regs[2]} Ah")
        p(f"  [E003] Bat nominal volt:    {bat_v} V")
        p(f"  [E004] Bat type:            {bat_type} = {BATTERY_TYPE.get(bat_type, f'?({bat_type})')}")
        vnames = [
            "OverVolt protect",    "ChgLimit volt",    "ConstChg volt",
            "ImprovChg volt",      "Float chg volt",   "ImprovChgBack volt",
            "OverDischgBack volt", "UnderVolt warn",   "OverDischg volt",
            "DischgLimit volt",
        ]
        for i, name in enumerate(vnames):
            raw = regs[5+i]
            v12 = raw * 0.1
            va  = v12 * v_factor
            p(f"  [E{5+i:03X}] {name:22s}: {raw:3d} -> {v12:.1f}V(12V) = {va:.1f}V({bat_v}V)")
        p(f"  [E00F] Dischg stop SOC:     {regs[15]}%")
        p(f"  [E010] OverDischg delay:    {regs[16]} s")
        p(f"  [E011] ConstChg time:       {regs[17]} min")
        p(f"  [E012] ImprovChg time:      {regs[18]} min  (Boost charge duration)")
        p(f"  [E013] ConstChg gap:        {regs[19]} days")
        p(f"  [E014] Temp comp coeff:     {regs[20]} mV/C/2")
        p(f"  [E015] Chg max temp:        {regs[21]} C")
    else:
        p(f"  [E000-E015] -> {status}")


def scan_inverter_settings(ser, addr):
    p()
    p("=" * 65)
    p("  P07 - INVERTER USER SETTINGS (0xE200 - 0xE222)")
    p("=" * 65)

    regs, status = read_regs(ser, addr, 0xE200, 22)
    if status == 'ok':
        pm = regs[1]
        op = regs[4]
        p(f"  [E200] RS485 addr:          {regs[0]}")
        p(f"  [E201] Parallel mode:       {pm} = {PARALLEL_MODE.get(pm, f'?({pm})')}")
        p(f"  [E202] Password:            {'(set)' if regs[2] > 0 else '(none)'}")
        p(f"  [E204] Output priority:     {op} = {OUTPUT_PRIORITY.get(op, f'?({op})')}  *** OUTPUT PRIORITY ***")
        p(f"  [E205] Grid input volt:     {regs[5] * 0.1:.1f} V")
        p(f"  [E206] Grid output volt:    {regs[6] * 0.1:.1f} V")
        p(f"  [E207] Grid freq set:       {regs[7] * 0.01:.2f} Hz")
        p(f"  [E208] Inv volt set:        {regs[8] * 0.1:.1f} V")
        p(f"  [E209] Inv freq set:        {regs[9] * 0.01:.2f} Hz")
        p(f"  [E20A] OutVolt range:       {regs[10]}")
        p(f"  [E20B] OutFreq range:       {regs[11]}")
        p(f"  [E20C] Max chg curr:        {regs[12]} A")
        p(f"  [E20D] Max line curr:       {regs[13]} A")
        p(f"  [E20E] Bat low volt:        {regs[14] * 0.1:.1f} V")
        p(f"  [E20F] Bat high volt:       {regs[15] * 0.1:.1f} V  (raw={regs[15]})")
        p(f"  [E210] Bat back volt:       {regs[16] * 0.1:.1f} V")
        p(f"  [E211] Fault shutdown:      {regs[17]}")
        p(f"  [E212] Grid chg enable:     {regs[18]}")
        p(f"  [E213] Aux power:           {regs[19]}")
        p(f"  [E214] Chg time setting:    {regs[20]}")
        p(f"  [E215] Dischg time setting: {regs[21]}")
    else:
        p(f"  [E200-E215] -> {status}")

    regs, status = read_regs(ser, addr, 0xE21B, 8)
    if status == 'ok':
        p(f"  [E21B] BMS protocol:        {regs[0]}")
        p(f"  [E21C] Max line curr:       {regs[1] * 0.1:.1f} A")
        p(f"  [E21D] Max line power:      {regs[2]}")
        p(f"  [E21E] Output phase set:    {regs[3]}")
        p(f"  [E21F] Gen work mode:       {regs[4]}")
        p(f"  [E220] Gen chg max curr:    {regs[5] * 0.1:.1f} A")
        p(f"  [E221] Gen rated power:     {regs[6]}")
    else:
        p(f"  [E21B-E221] -> {status}")


def scan_energy_storage_factory(ser, addr):
    p()
    p("=" * 65)
    p("  ENERGY STORAGE FACTORY SETTINGS (0xEA80 - 0xEA9F)")
    p("=" * 65)
    regs, status = read_regs(ser, addr, 0xEA80, 32)
    if status == 'ok':
        for i, v in enumerate(regs):
            p(f"  [EA{0x80+i:02X}]  {v:6d}  0x{v:04X}  {v:016b}b")
    else:
        p(f"  [EA80-EA9F] -> {status}")


def scan_fault_records(ser, addr):
    p()
    p("=" * 65)
    p("  FAULT RECORDS (0xF800 - first record)")
    p("=" * 65)
    regs, status = read_regs(ser, addr, 0xF800, 16)
    if status == 'ok':
        fault_code = regs[0]
        if fault_code == 0:
            p("  [F800] No fault records (fault code = 0)")
        else:
            p(f"  [F800] Fault code: {fault_code}")
            p(f"  Data: {[f'{v:#06x}' for v in regs[1:16]]}")
    else:
        p(f"  [F800] -> {status}")


# --- Main ---------------------------------------------------------------------

AREAS = {
    'product':  [scan_product_info],
    'live':     [scan_live_dc, scan_live_ac, scan_live_rtc, scan_energy_storage_live],
    'stats':    [scan_stats_today, scan_stats_history],
    'settings': [scan_battery_settings, scan_inverter_settings,
                 scan_energy_storage_factory],
    'faults':   [scan_fault_records],
}
AREAS['all'] = (AREAS['product'] + AREAS['live'] +
                AREAS['stats'] + AREAS['settings'] + AREAS['faults'])


def main():
    setup_logger()
    parser = argparse.ArgumentParser(description="SRNE Full Register Scanner v1.0")
    parser.add_argument("port",   nargs="?", default="/dev/ttyUSB1")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=1)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--area", choices=list(AREAS.keys()), default="all")
    args = parser.parse_args()

    p("=" * 65)
    p("  SRNE Energy Storage Inverter - Full Register Scanner v1.0")
    p("  Protocol: MODBUS Energy Storage Inverter v1.96 (2024)")
    p(f"  Log: {LOG_FILE}")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p("=" * 65)
    p(f"  Port: {args.port} | Baud: {args.baud} | Addr: {args.addr}")
    p(f"  Area: {args.area}")

    try:
        ser = serial.Serial(args.port, args.baud, bytesize=8,
                            parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)
        p(f"  Port OK: {args.port}")
    except serial.SerialException as e:
        p(f"  FAIL: {e}")
        sys.exit(1)

    time.sleep(0.3)
    for fn in AREAS[args.area]:
        fn(ser, args.addr)
        time.sleep(0.2)

    p()
    p("=" * 65)
    p("  Scanare completa.")
    p(f"  Log salvat: {LOG_FILE}")
    p("=" * 65)
    ser.close()

if __name__ == "__main__":
    main()
