#!/usr/bin/env python3
"""
srne_modbus.py v2.0.0 - SRNE Invertor Modbus RTU -> MQTT -> Home Assistant
===========================================================================
Dispozitiv testat: Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H
  Serial: SR-2211150019-300917
  Firmware: APP V6.64 (Jun 2 2022), Boot V2.01, HW V2.00
  ProductType: 4 = All-in-one solar charger inverter
Interfata: Port USB-B (mufa patrata) -> CH340 -> /dev/ttyUSB*
Protocol:  Modbus RTU, addr=1, FC03 read, FC06/FC16 write, 9600 8N1

===============================================================================
REGISTER MAP - CONFIRMAT PE FIRMWARE HF2450S80H (scan 2026-05-02)
===============================================================================

PRODUCT INFO (citit o data la startup):
  0x000A x 2:  ProductType, MinorVersion
  0x0014 x 4:  APP version (664=V6.64), Boot version (201=V2.01), HW versions
  0x001A x 2:  RS485 addr, ModelCode
  0x0021 x 20: CPU build time string ('Jun  2 2022 11:16:45')
  0x0035 x 20: Serial number string ('SR-2211150019-300917')

LIVE DC DATA (fast poll, max 15 regs!):
  0x0100 x 15: SOC, Vbat, Ibat(signed x0.1A), [Tbat=0 pe HF2450S80H],
               Vpv1(x0.1V), Ipv1(x0.1A), Ppv1(W), PvTotalPwr(W),
               ChargeState@0x010B
  Nota: 0x010F+ (Pv2) -> exception pe HF2450S80H, max 15 regs!

LIVE AC DATA (fast poll) - FORMAT VECHI firmware:
  0x0204 x 31: (0x0204-0x0222 functional, 0x0210+ gol sau incompatibil)
               0x020C-0x020E = RTC (confirmat)
               0x0216=InvVoltA, 0x0218=InvFreq, 0x0219=LoadCurrA
               0x021B=LoadActivePow, 0x021C=LoadApparentPow
               0x021E=LineChgCurr(AC->bat), 0x021F=LoadRatio%
               0x0220-0x0222=Temp DC/AC/Trafo
  Nota: 0x0210 x 32 (v1.96) -> exception pe HF2450S80H!
  Nota: 0x7100 (energy storage live) -> exception, nu exista!

STATS ZILNIC + CUMULATIV (fast poll):
  0xF02C x 8:  F02D=BatChg azi(Ah), F02E=BatDchg azi(Ah)
               F02F=PV azi(kWh x0.1), F030=Consum azi(kWh x0.1)
               F031=zile operare, F032-F033=GridEnergy total(32-bit)
  0xF034 x 8:  F034-F035=BatChgTotal(Ah, 32-bit LE)  <- 17572 Ah=DessMonitor!
               F036-F037=BatDchgTotal(Ah, 32-bit LE)
               F038-F039=PV Total(kWh x0.1, 32-bit LE)
               F03A-F03B=Load Total(kWh x0.1, 32-bit LE)
  0xF03C x 6:  F03C=GridChg azi(Ah), F03D=GridLoad azi(kWh)
               F03E=InvWork azi(min), F03F=GridWork azi(min)

STATS ISTORICE 7 ZILE (slow poll - 1/ora):
  0xF000 x 28: F000=PV ieri, F001=PV acum2zile, ..., F006=PV acum7zile (x0.1kWh)
               F007-F00D=BatChg 7 zile(Ah)
               F00E-F014=BatDchg 7 zile(Ah)
               F015-F01B=GridChg 7 zile (toate 0 pt off-grid)

SETARI BATERIE (slow poll - max 5/10 regs per cerere, >22 -> timeout!):
  0xE000 x 5:  E001=PvChgCurrMax(A), E002=BatCap(Ah), E003=BatNomVolt(V)
               E004=BatType (v1.96) / MachineState (v3.9 firmware)
  0xE005 x 10: praguri tensiune (x0.1V sistem 12V, nmultit cu Vnom/12 pt real)

SETARI INVERTOR (individual, >22 -> timeout!):
  0xE204 x 1:  OutputPriority per v1.96 standard
  0xE20F x 1:  BatHighVolt (v1.96) / OutputPriority (firmware vechi HF2450S80H)
               Confirmat: E20F=3=Solar only/Off-grid pe HF2450S80H

WORK TIME (slow poll):
  0xF04A x 2:  F04A=InvWorkTotal(h), F04B=GridWorkTotal(h)

===============================================================================
HA Energy Dashboard: pv_energy_total_kwh, load_energy_total_kwh,
  battery_charge_total_ah, battery_discharge_total_ah

Changelog:
  v2.0.0 - IMPLEMENTARE COMPLETA bazata pe scan confirmat (2026-05-02)
           CONFIRMAT: serial SR-2211150019-300917, APP V6.64, Boot V2.01
           CONFIRMAT: F02D/F02E bat chg/dischg azi (Ah)
           CONFIRMAT: F034-F037 total Ah 32-bit = 17572 Ah (DessMonitor!)
           CONFIRMAT: F000-F01B istoricul 7 zile PV+BatChg+BatDchg
           CONFIRMAT: F03E inv work azi (min), F04A/F04B work total
           FIX: E register reads <= 10 regs (>22 -> timeout pe HF2450S80H)
           FIX: Output priority: incearca E204 (v1.96), fallback E20F (fw vechi)
           FIX: ChargeState la 0x010B conform v1.96
           FIX: E004 = BatType (v1.96) / MachineState (v3.9 fw)
           ADAUGAT: serial_number, fw_app_version, fw_boot_version
           ADAUGAT: bat_charge/discharge_today_ah, _total_ah (total_increasing)
           ADAUGAT: inv_work_today_min, inv_work_total_h
           ADAUGAT: praguri tensiune baterie (E005-E00E)
           ELIMINAT: interpretari gresite anterioare
  v1.3.x - versiuni intermediare cu fix-uri partiale
  v1.0.0 - initial

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import json
import logging
import os
import struct
import sys
import time
from datetime import datetime, date

import paho.mqtt.client as mqtt
import serial

# ─── Config ───────────────────────────────────────────────────────────────────

DEFAULTS = {
    "serial_port":         "/dev/ttyUSB1",
    "modbus_address":      1,
    "poll_interval":       30,
    "mqtt_host":           "core-mosquitto",
    "mqtt_port":           1883,
    "mqtt_user":           "mqtt_local",
    "mqtt_password":       "mqtt2026vidra",
    "mqtt_topic_prefix":   "srne",
    "ha_discovery_prefix": "homeassistant",
    "log_level":           "INFO",
}

def load_config() -> dict:
    cfg = dict(DEFAULTS)
    options_path = "/data/options.json"
    if os.path.exists(options_path):
        try:
            with open(options_path) as f:
                cfg.update(json.load(f))
        except Exception as e:
            print(f"[WARN] options.json: {e}")
    return cfg

# ─── Logging ──────────────────────────────────────────────────────────────────

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "srne_modbus.log")

def setup_logging(level_str: str):
    level = getattr(logging, level_str.upper(), logging.INFO)
    fmt   = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                              datefmt="%Y-%m-%d %H:%M:%S")
    root  = logging.getLogger()
    root.setLevel(level)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    root.addHandler(ch)
    try:
        fh = logging.FileHandler(LOG_FILE, mode="a", encoding="utf-8")
        fh.setFormatter(fmt)
        root.addHandler(fh)
    except Exception as e:
        logging.warning(f"Nu pot deschide log file {LOG_FILE}: {e}")

# ─── CRC16 Modbus ─────────────────────────────────────────────────────────────

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def _build_fc03(addr, reg_start, count):
    pdu = struct.pack(">BBHH", addr, 0x03, reg_start, count)
    return pdu + struct.pack("<H", _crc16(pdu))

def _build_fc06(addr, reg, value):
    pdu = struct.pack(">BBHH", addr, 0x06, reg, value & 0xFFFF)
    return pdu + struct.pack("<H", _crc16(pdu))

def _build_fc16(addr, reg_start, values):
    count = len(values)
    pdu = struct.pack(">BBHHB", addr, 0x10, reg_start, count, count * 2)
    for v in values:
        pdu += struct.pack(">H", v & 0xFFFF)
    return pdu + struct.pack("<H", _crc16(pdu))

def _to_signed16(v: int) -> int:
    return v if v < 0x8000 else v - 0x10000

def _u32_le(lo: int, hi: int) -> int:
    """32-bit little-endian: low register la adresa mica, high la adresa mare."""
    return (hi << 16) | lo

def _decode_string_regs(regs) -> str:
    """SRNE string: low byte of each register = char."""
    chars = []
    for r in regs:
        lo = r & 0xFF
        if lo == 0:
            break
        chars.append(chr(lo) if 32 <= lo < 127 else '?')
    return ''.join(chars).strip()

# ─── Receive cu filtrare adresa slave ─────────────────────────────────────────

def _recv_slave_frame(ser, slave_addr: int, expected_regs: int, timeout=3.0):
    buf = bytearray()
    start = time.time()
    ignored = 0
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        i = 0
        while i < len(buf):
            if buf[i] != slave_addr:
                ignored += 1
                i += 1
                continue
            rest = buf[i:]
            if len(rest) >= 3 and rest[1] == 0x03:
                bc = rest[2]
                if bc != expected_regs * 2:
                    i += 1
                    continue
                total = 3 + bc + 2
                if len(rest) < total:
                    break
                frame = bytes(rest[:total])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0:
                        logging.debug(f"Bus: ignorat {ignored}b")
                    return frame, False
                i += 1
                continue
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0:
                        logging.debug(f"Bus: ignorat {ignored}b")
                    return frame, True
                i += 1
                continue
            i += 1
    if ignored > 0:
        logging.debug(f"Bus: ignorat {ignored}b (timeout)")
    return None, False

# ─── Modbus RTU ───────────────────────────────────────────────────────────────

class ModbusRTU:
    def __init__(self, port, baudrate=9600, device_addr=1):
        self.port        = port
        self.device_addr = device_addr
        self._baudrate   = baudrate
        self._ser        = None

    def connect(self):
        self._ser = serial.Serial(self.port, self._baudrate, bytesize=8,
                                  parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)
        logging.info(f"Serial OK: {self.port} @ {self._baudrate} bps")

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_registers(self, reg_start: int, count: int) -> list | None:
        request = _build_fc03(self.device_addr, reg_start, count)
        self._ser.reset_input_buffer()
        time.sleep(0.06)
        self._ser.write(request)
        frame, is_exc = _recv_slave_frame(self._ser, self.device_addr, count)
        if frame is None:
            logging.warning(f"Timeout 0x{reg_start:04X}x{count}")
            return None
        if is_exc:
            logging.warning(f"Exception 0x{reg_start:04X}: 0x{frame[2]:02X}")
            return None
        return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(count)]

    def write_register(self, reg: int, value: int) -> bool:
        request = _build_fc06(self.device_addr, reg, value)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        resp = self._ser.read(8)
        if len(resp) == 8 and struct.unpack("<H", resp[-2:])[0] == _crc16(resp[:-2]):
            return True
        logging.warning(f"FC06 0x{reg:04X}={value}: fail")
        return False

    def write_registers(self, reg_start: int, values: list) -> bool:
        request = _build_fc16(self.device_addr, reg_start, values)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        resp = self._ser.read(8)
        if len(resp) == 8 and struct.unpack("<H", resp[-2:])[0] == _crc16(resp[:-2]):
            return True
        logging.warning(f"FC16 0x{reg_start:04X}: fail")
        return False

    def read_rtc(self) -> datetime | None:
        regs = self.read_registers(0x020C, 3)
        if not regs:
            return None
        try:
            r0, r1, r2 = regs
            return datetime((r0 >> 8) + 2002, r0 & 0xFF, r1 >> 8,
                            r1 & 0xFF, r2 >> 8, r2 & 0xFF)
        except Exception as e:
            logging.warning(f"RTC parse: {e}")
            return None

    def sync_rtc(self) -> bool:
        now = datetime.now()
        yy  = now.year - 2002
        ok = self.write_registers(0x020C, [(yy << 8) | now.month,
                                           (now.day << 8) | now.hour,
                                           (now.minute << 8) | now.second])
        logging.info(f"RTC sync: {'OK' if ok else 'FAIL'} -> {now.strftime('%Y-%m-%d %H:%M:%S')}")
        return ok

    def check_and_sync_rtc(self, drift_s: int = 60) -> bool:
        inv = self.read_rtc()
        now = datetime.now()
        if inv is None:
            logging.warning("RTC check: nu pot citi")
            return False
        d = abs((now - inv).total_seconds())
        logging.info(f"RTC check: inv={inv.strftime('%H:%M:%S')} sys={now.strftime('%H:%M:%S')} drift={d:.0f}s")
        if d > drift_s:
            logging.info(f"RTC drift {d:.0f}s > {drift_s}s -> sync")
            return self.sync_rtc()
        logging.info(f"RTC drift {d:.0f}s ok")
        return False

# ─── Enumerari ────────────────────────────────────────────────────────────────

CHARGE_STATE = {
    0:"Off", 1:"Quick charge", 2:"Const voltage", 3:"Equalize",
    4:"Float", 5:"Boost", 6:"Li activate", 7:"Const current",
    8:"Full", 9:"Current limit",
}

MACHINE_STATE = {
    0:"Standby", 1:"Running", 2:"SW startup", 3:"Starting",
    4:"AC mode", 5:"Inverter mode", 6:"ECO mode",
    7:"Fault", 8:"Shutdown", 9:"Running (inv)",
}

BATTERY_TYPE = {
    0:"User define", 1:"SLD", 2:"FLD", 3:"GEL",
    4:"LiFePO4 x14", 5:"LiFePO4 x15", 6:"LiFePO4 x16",
    7:"LiFePO4 x7",  8:"LiFePO4 x8",  9:"LiFePO4 x9",
    10:"Ternary x7", 11:"Ternary x8", 12:"Ternary x13", 13:"Ternary x14",
}

OUTPUT_PRIORITY = {
    0:"Solar first", 1:"Grid first", 2:"SBU priority",
    3:"Solar only / Off-grid",
}

# ─── Parsare registri ─────────────────────────────────────────────────────────

def parse_0100(regs: list) -> dict:
    """0x0100 x 15. Confirmat pe HF2450S80H. Max 15 regs (0x010F+ -> exception)."""
    def r(a): return regs[a - 0x0100] if 0 <= a - 0x0100 < len(regs) else 0
    ibat = _to_signed16(r(0x0102))
    cs   = r(0x010B) & 0xFF    # ChargeState la 0x010B per v1.96
    return {
        "battery_soc":         r(0x0100) & 0xFF,
        "battery_voltage":     round(r(0x0101) * 0.1, 1),
        "battery_current":     round(ibat * 0.1, 1),
        "pv_voltage":          round(r(0x0107) * 0.1, 1),
        "pv_current":          round(r(0x0108) * 0.1, 1),
        "pv_power":            r(0x0109),
        "pv_total_power":      r(0x010A),
        "battery_charge_step": CHARGE_STATE.get(cs, f"?({cs})"),
    }


def parse_0204(regs: list) -> dict:
    """
    0x0204 x 31. Format vechi firmware HF2450S80H.
    0x0210 x 32 (v1.96) da exception pe HF2450S80H!
    """
    def r(a): return regs[a - 0x0204] if 0 <= a - 0x0204 < len(regs) else 0
    ms  = r(0x0209) & 0xFF
    r0, r1, r2 = r(0x020C), r(0x020D), r(0x020E)
    pac = r(0x021B)
    pap = r(0x021C)
    try:
        rtc = f"{(r0>>8)+2002:04d}-{r0&0xFF:02d}-{r1>>8:02d}T{r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}"
    except Exception:
        rtc = "invalid"
    return {
        "machine_state_code":  ms,
        "machine_state":       MACHINE_STATE.get(ms, f"?({ms})"),
        "rtc_datetime":        rtc,
        "load_ratio":          r(0x021F),
        "line_chg_current":    round(r(0x021E) * 0.1, 1),
        "ac_output_voltage":   round(r(0x0216) * 0.1, 1),
        "ac_output_frequency": round(r(0x0218) * 0.01, 2),
        "ac_output_current":   round(r(0x0219) * 0.1, 1),
        "ac_active_power":     pac,
        "ac_apparent_power":   pap,
        "power_factor":        round(pac / pap, 3) if pap else 0.0,
        "temp_dc_side":        round(_to_signed16(r(0x0220)) * 0.1, 1),
        "temp_ac_side":        round(_to_signed16(r(0x0221)) * 0.1, 1),
        "temp_transformer":    round(_to_signed16(r(0x0222)) * 0.1, 1),
    }


def parse_F02C(regs: list) -> dict:
    """0xF02C x 8. Confirmat pe HF2450S80H."""
    def r(i): return regs[i] if i < len(regs) else 0
    return {
        "pv_to_grid_today_kwh":       round(r(0) * 0.1, 1),
        "battery_charge_today_ah":    r(1),        # F02D ***
        "battery_discharge_today_ah": r(2),        # F02E ***
        "pv_energy_today_kwh":        round(r(3) * 0.1, 1),   # F02F
        "load_energy_today_kwh":      round(r(4) * 0.1, 1),   # F030
        "operating_days":             r(5),
    }


def parse_F034(regs: list) -> dict:
    """
    0xF034 x 8. 32-bit LE (low reg first, high reg second).
    F034-F035: BatChgTotal Ah = 17572 Ah (confirmat DessMonitor!)
    """
    def r(i): return regs[i] if i < len(regs) else 0
    return {
        "battery_charge_total_ah":    _u32_le(r(0), r(1)),               # F034-F035 ***
        "battery_discharge_total_ah": _u32_le(r(2), r(3)),               # F036-F037 ***
        "pv_energy_total_kwh":        round(_u32_le(r(4), r(5)) * 0.1, 1),   # F038-F039
        "load_energy_total_kwh":      round(_u32_le(r(6), r(7)) * 0.1, 1),   # F03A-F03B
    }


def parse_F03C(regs: list) -> dict:
    """0xF03C x 6. Confirmat pe HF2450S80H."""
    def r(i): return regs[i] if i < len(regs) else 0
    return {
        "grid_charge_today_ah":  r(0),
        "grid_load_today_kwh":   round(r(1) * 0.1, 1),
        "inv_work_today_min":    r(2),   # F03E ***
        "grid_work_today_min":   r(3),
    }


def parse_F000_history(regs: list) -> dict:
    """
    0xF000 x 28. F000=PV ieri, F001=PV acum2zile, ..., F006=PV acum7zile.
    F007-F00D=BatChg 7 zile. F00E-F014=BatDchg 7 zile.
    """
    def r(i): return regs[i] if i < len(regs) else 0
    result = {}
    labels = ["yesterday", "2d_ago", "3d_ago", "4d_ago", "5d_ago", "6d_ago", "7d_ago"]
    for i, lbl in enumerate(labels):
        result[f"pv_energy_{lbl}_kwh"]  = round(r(i)     * 0.1, 1)
        result[f"bat_chg_{lbl}_ah"]     = r(7  + i)
        result[f"bat_dischg_{lbl}_ah"]  = r(14 + i)
    return result


def read_product_info(mb: ModbusRTU) -> dict:
    """Citeste informatii produs o data la startup."""
    info = {}
    r = mb.read_registers(0x000A, 2)
    if r:
        info["product_type_code"] = r[1]
    time.sleep(0.1)
    r = mb.read_registers(0x0014, 3)
    if r:
        info["fw_app_version"]  = round(r[0] / 100, 2)   # 664 -> 6.64
        info["fw_boot_version"] = round(r[1] / 100, 2)   # 201 -> 2.01
        info["hw_ctrl_version"] = round(r[2] / 100, 2)
    time.sleep(0.1)
    r = mb.read_registers(0x001A, 2)
    if r:
        info["rs485_address"] = r[0]
        info["model_code"]    = r[1]
    time.sleep(0.1)
    r = mb.read_registers(0x0021, 20)
    if r:
        info["cpu_build_time"] = _decode_string_regs(r)
    time.sleep(0.1)
    r = mb.read_registers(0x0035, 20)
    if r:
        info["serial_number"] = _decode_string_regs(r)
    time.sleep(0.1)
    if info:
        logging.info(f"Product: SN={info.get('serial_number')} "
                     f"APP=V{info.get('fw_app_version')} "
                     f"Boot=V{info.get('fw_boot_version')}")
    return info


def read_slow_registers(mb: ModbusRTU, state_cache: dict) -> dict:
    """
    Registri cititi o data pe ora.
    ATENTIE: E-register blocuri > 10 dau TIMEOUT pe HF2450S80H!
    """
    result = {}

    # Output priority: E204 (v1.96 standard) sau E20F (firmware vechi)
    # Pe HF2450S80H: E20F=3=Solar only/Off-grid (confirmat)
    for reg_addr in [0xE204, 0xE20F]:
        r = mb.read_registers(reg_addr, 1)
        if r is not None and r[0] <= 3:
            op_val = OUTPUT_PRIORITY.get(r[0], f"?({r[0]})")
            if state_cache.get("output_priority") != op_val:
                logging.info(f"Output priority [0x{reg_addr:04X}]: {op_val} (raw={r[0]})")
            result["output_priority"] = op_val
            state_cache["output_priority"] = op_val
            break
        time.sleep(0.1)
    if "output_priority" not in result and "output_priority" in state_cache:
        result["output_priority"] = state_cache["output_priority"]

    # Setari baterie E000-E004 (max 5 regs! E000 x 22 -> timeout)
    r = mb.read_registers(0xE000, 5)
    if r is not None:
        bat_v    = r[3]
        bat_type = r[4]
        v_factor = bat_v / 12.0 if bat_v > 0 else 2.0
        bat_data = {
            "bat_pv_chg_max_a":   r[1],
            "bat_nominal_cap_ah": r[2],
            "bat_nominal_volt_v": bat_v,
            "bat_type_code":      bat_type,
            "bat_type":           BATTERY_TYPE.get(bat_type, f"?({bat_type})"),
        }
        result.update(bat_data)
        if "bat_nominal_volt_v" not in state_cache:
            logging.info(f"Battery: {bat_data['bat_type']} "
                         f"{r[2]}Ah {bat_v}V PVmax={r[1]}A")
        state_cache.update(bat_data)
        time.sleep(0.1)

        # Praguri tensiune E005-E00E (max 10 regs)
        rv = mb.read_registers(0xE005, 10)
        if rv is not None:
            vn = ["bat_over_volt", "bat_chg_limit_volt", "bat_const_chg_volt",
                  "bat_improve_chg_volt", "bat_float_chg_volt",
                  "bat_improve_chg_back_volt", "bat_over_dischg_back_volt",
                  "bat_under_volt", "bat_over_dischg_volt", "bat_dischg_limit_volt"]
            for i, name in enumerate(vn):
                val = round(rv[i] * 0.1 * v_factor, 1)
                result[name] = val
            if "bat_over_volt" not in state_cache:
                logging.info(f"Bat volts: OV={result['bat_over_volt']}V "
                             f"Float={result['bat_float_chg_volt']}V "
                             f"OD={result['bat_over_dischg_volt']}V")
            state_cache.update({k: result[k] for k in vn if k in result})
        time.sleep(0.1)
    elif "bat_nominal_volt_v" in state_cache:
        for k in ("bat_pv_chg_max_a", "bat_nominal_cap_ah", "bat_nominal_volt_v",
                  "bat_type", "bat_type_code", "bat_over_volt", "bat_float_chg_volt",
                  "bat_over_dischg_volt"):
            if k in state_cache:
                result[k] = state_cache[k]

    # Istoricul 7 zile F000-F01B (28 regs, confirmat)
    r = mb.read_registers(0xF000, 28)
    if r is not None:
        hist = parse_F000_history(r)
        result.update(hist)
        state_cache.update(hist)
    else:
        for k, v in state_cache.items():
            if "yesterday" in k or "d_ago" in k:
                result[k] = v
    time.sleep(0.1)

    # Work time total F04A-F04B
    r = mb.read_registers(0xF04A, 2)
    if r is not None:
        result["inv_work_total_h"]  = r[0]
        result["grid_work_total_h"] = r[1]
        state_cache["inv_work_total_h"]  = r[0]
        state_cache["grid_work_total_h"] = r[1]
    elif "inv_work_total_h" in state_cache:
        result["inv_work_total_h"]  = state_cache["inv_work_total_h"]
        result["grid_work_total_h"] = state_cache.get("grid_work_total_h", 0)
    time.sleep(0.1)

    return result


def read_all(mb: ModbusRTU) -> dict | None:
    """Citeste toti registrii fast poll (30s). None = eroare critica."""
    result = {"timestamp": datetime.now().isoformat()}

    r0100 = mb.read_registers(0x0100, 15)   # max 15! 0x010F+ -> exception
    if r0100 is None:
        return None
    result.update(parse_0100(r0100))
    time.sleep(0.15)

    r0204 = mb.read_registers(0x0204, 31)   # format vechi, confirmat
    if r0204:
        result.update(parse_0204(r0204))
    time.sleep(0.15)

    rF02C = mb.read_registers(0xF02C, 8)    # bat Ah azi + PV/load kWh azi
    if rF02C:
        result.update(parse_F02C(rF02C))
    time.sleep(0.1)

    rF034 = mb.read_registers(0xF034, 8)    # total Ah + total kWh (32-bit)
    if rF034:
        result.update(parse_F034(rF034))
    time.sleep(0.1)

    rF03C = mb.read_registers(0xF03C, 6)    # work time azi
    if rF03C:
        result.update(parse_F03C(rF03C))
    time.sleep(0.1)

    result["fault_active"] = False
    result["fault_code"]   = 0

    return result

# ─── MQTT + HA Auto-Discovery ─────────────────────────────────────────────────

SENSORS = [
    # key                           unit   dc              name                          icon                       ent_cat      prec
    # Baterie
    ("battery_soc",                "%",   "battery",      "SOC Baterie",                "mdi:battery",             None,        0),
    ("battery_voltage",            "V",   "voltage",      "Tensiune Baterie",           "mdi:battery-charging",    None,        1),
    ("battery_current",            "A",   "current",      "Curent Baterie",             "mdi:current-dc",          None,        1),
    ("battery_charge_today_ah",    "Ah",  None,           "Incarcare Bat Azi",          "mdi:battery-charging",    None,        0),
    ("battery_discharge_today_ah", "Ah",  None,           "Descarcare Bat Azi",         "mdi:battery-arrow-down",  None,        0),
    ("battery_charge_total_ah",    "Ah",  None,           "Incarcare Bat Total",        "mdi:battery-plus",        None,        0),
    ("battery_discharge_total_ah", "Ah",  None,           "Descarcare Bat Total",       "mdi:battery-minus",       None,        0),
    # PV
    ("pv_voltage",                 "V",   "voltage",      "Tensiune PV",                "mdi:solar-panel",         None,        1),
    ("pv_current",                 "A",   "current",      "Curent PV",                  "mdi:solar-panel",         None,        1),
    ("pv_power",                   "W",   "power",        "Putere PV",                  "mdi:solar-power",         None,        0),
    ("pv_energy_today_kwh",        "kWh", "energy",       "Energie PV Azi",             "mdi:solar-power",         None,        1),
    ("pv_energy_total_kwh",        "kWh", "energy",       "Energie PV Total",           "mdi:solar-power",         None,        1),
    # AC Output
    ("ac_output_voltage",          "V",   "voltage",      "Tensiune AC Out",            "mdi:power-plug",          None,        1),
    ("ac_output_frequency",        "Hz",  "frequency",    "Frecventa AC Out",           "mdi:sine-wave",           None,        2),
    ("ac_output_current",          "A",   "current",      "Curent AC Out",              "mdi:current-ac",          None,        1),
    ("ac_active_power",            "W",   "power",        "Putere Activa AC",           "mdi:lightning-bolt",      None,        0),
    ("ac_apparent_power",          "VA",  None,           "Putere Aparenta AC",         "mdi:lightning-bolt",      None,        0),
    ("power_factor",               None,  "power_factor", "Factor Putere",              "mdi:angle-acute",         None,        3),
    ("load_ratio",                 "%",   None,           "Sarcina %",                  "mdi:gauge",               None,        0),
    ("line_chg_current",           "A",   "current",      "Curent Incarcare Retea",     "mdi:transmission-tower",  None,        1),
    ("load_energy_today_kwh",      "kWh", "energy",       "Consum Sarcina Azi",         "mdi:home-lightning-bolt", None,        1),
    ("load_energy_total_kwh",      "kWh", "energy",       "Consum Sarcina Total",       "mdi:home-lightning-bolt", None,        1),
    # Temperaturi
    ("temp_dc_side",               "°C",  "temperature",  "Temp DC Side",               "mdi:thermometer",         "diagnostic",1),
    ("temp_ac_side",               "°C",  "temperature",  "Temp AC Side",               "mdi:thermometer",         "diagnostic",1),
    ("temp_transformer",           "°C",  "temperature",  "Temp Trafo",                 "mdi:thermometer",         "diagnostic",1),
    # Stare
    ("battery_charge_step",        None,  None,           "Etapa Incarcare",            "mdi:battery-charging",    "diagnostic",None),
    ("machine_state",              None,  None,           "Stare Invertor",             "mdi:information",         "diagnostic",None),
    ("output_priority",            None,  None,           "Prioritate Iesire",          "mdi:priority-high",       "diagnostic",None),
    # Timpi functionare
    ("inv_work_today_min",         "min", None,           "Inv Work Azi",               "mdi:timer",               "diagnostic",0),
    ("inv_work_total_h",           "h",   None,           "Inv Work Total",             "mdi:timer",               "diagnostic",0),
    # Produs + Firmware
    ("fw_app_version",             None,  None,           "Firmware APP",               "mdi:chip",                "diagnostic",2),
    ("fw_boot_version",            None,  None,           "Firmware Boot",              "mdi:chip",                "diagnostic",2),
    ("serial_number",              None,  None,           "Serial Number",              "mdi:barcode",             "diagnostic",None),
    # Setari baterie (slow poll, static)
    ("bat_pv_chg_max_a",           "A",   None,           "PV Curent Max Incarcare",    "mdi:solar-panel",         "diagnostic",0),
    ("bat_nominal_cap_ah",         "Ah",  None,           "Capacitate Nominala Bat",    "mdi:battery",             "diagnostic",0),
    ("bat_nominal_volt_v",         "V",   None,           "Tensiune Nominala Bat",      "mdi:battery",             "diagnostic",0),
    ("bat_type",                   None,  None,           "Tip Baterie",                "mdi:battery-heart",       "diagnostic",None),
    ("bat_float_chg_volt",         "V",   "voltage",      "Tensiune Float",             "mdi:battery-charging",    "diagnostic",1),
    ("bat_over_dischg_volt",       "V",   "voltage",      "Tensiune OverDischg",        "mdi:battery-alert",       "diagnostic",1),
]

TOTAL_INCREASING_KEYS = {
    "pv_energy_total_kwh", "load_energy_total_kwh",
    "battery_charge_total_ah", "battery_discharge_total_ah",
    "inv_work_total_h",
}
MEASUREMENT_KEYS = {
    "battery_charge_today_ah", "battery_discharge_today_ah",
    "pv_energy_today_kwh", "load_energy_today_kwh",
    "inv_work_today_min",
}
NO_STATE_CLASS_KEYS = {
    "fw_app_version", "fw_boot_version", "serial_number",
    "bat_pv_chg_max_a", "bat_nominal_cap_ah", "bat_nominal_volt_v", "bat_type",
}


def _make_device(product_info: dict) -> dict:
    sn  = product_info.get("serial_number", "")
    app = product_info.get("fw_app_version", "?")
    return {
        "identifiers":  [f"srne_{sn}" if sn else "srne_hf2450s80h"],
        "name":         "SRNE Invertor",
        "model":        "HF2450S80H (Easun ISI Max II 3.6kW/24V)",
        "manufacturer": "SRNE Solar",
        "serial_number":sn,
        "sw_version":   f"APP V{app}",
    }


def publish_discovery(client, cfg: dict, product_info: dict):
    prefix = cfg["ha_discovery_prefix"]
    state  = f"{cfg['mqtt_topic_prefix']}/state"
    device = _make_device(product_info)

    for key, unit, dc, name, icon, ent_cat, precision in SENSORS:
        p = {
            "name":           name,
            "unique_id":      f"srne_{key}",
            "state_topic":    state,
            "value_template": f"{{{{ value_json.{key} }}}}",
            "device":         device,
            "icon":           icon,
        }
        if unit:      p["unit_of_measurement"] = unit
        if dc:        p["device_class"] = dc
        if ent_cat:   p["entity_category"] = ent_cat
        if precision is not None:
            p["suggested_display_precision"] = precision

        if key in TOTAL_INCREASING_KEYS:
            p["state_class"] = "total_increasing"
        elif key in MEASUREMENT_KEYS:
            p["state_class"] = "measurement"
        elif key not in NO_STATE_CLASS_KEYS and unit in ("W","VA","V","A","%","Hz","°C","Ah","h","min"):
            p["state_class"] = "measurement"

        client.publish(f"{prefix}/sensor/srne_{key}/config", json.dumps(p), retain=True)

    client.publish(f"{prefix}/binary_sensor/srne_fault_active/config", json.dumps({
        "name":            "Fault Activ Invertor",
        "unique_id":       "srne_fault_active",
        "state_topic":     state,
        "value_template":  "{{ 'ON' if value_json.fault_active else 'OFF' }}",
        "device_class":    "problem",
        "device":          device,
        "entity_category": "diagnostic",
    }), retain=True)

    logging.info("HA auto-discovery publicat.")


def publish_state(client, topic_prefix: str, data: dict):
    client.publish(f"{topic_prefix}/state", json.dumps(data, default=str), retain=False)

# ─── Main ─────────────────────────────────────────────────────────────────────

SLOW_POLL_INTERVAL = 3600
RTC_CHECK_HOUR     = 0
RTC_CHECK_MINUTE   = 5
RTC_DRIFT_THRESH   = 60


def main():
    cfg = load_config()
    setup_logging(cfg.get("log_level", "INFO"))

    logging.info("=" * 58)
    logging.info("  SRNE Invertor Modbus v2.0.0")
    logging.info(f"  Log: {LOG_FILE}")
    logging.info(f"  Port: {cfg['serial_port']} | Addr: {cfg['modbus_address']} | Poll: {cfg['poll_interval']}s")
    logging.info(f"  RTC check: zilnic {RTC_CHECK_HOUR:02d}:{RTC_CHECK_MINUTE:02d}, sync daca drift > {RTC_DRIFT_THRESH}s")
    logging.info("=" * 58)

    mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="srne_invertor_addon")
    mq.username_pw_set(cfg["mqtt_user"], cfg["mqtt_password"])
    connected = False

    def on_connect(c, u, f, rc, p=None):
        nonlocal connected
        connected = (rc == 0)
        logging.info(f"MQTT {'OK' if connected else f'FAIL rc={rc}'}")

    def on_disconnect(c, u, rc, p=None):
        nonlocal connected
        connected = False
        logging.warning("MQTT deconectat")

    mq.on_connect    = on_connect
    mq.on_disconnect = on_disconnect

    while True:
        try:
            mq.connect(cfg["mqtt_host"], cfg["mqtt_port"], keepalive=60)
            break
        except Exception as e:
            logging.error(f"MQTT: {e}. Retry 10s...")
            time.sleep(10)

    mq.loop_start()
    time.sleep(2)

    mb = ModbusRTU(cfg["serial_port"], device_addr=cfg["modbus_address"])
    while True:
        try:
            mb.connect()
            break
        except serial.SerialException as e:
            logging.error(f"Serial: {e}. Retry 15s...")
            time.sleep(15)

    # Product info (startup)
    logging.info("Citire product info (serial, firmware)...")
    product_info = read_product_info(mb)

    if connected:
        publish_discovery(mq, cfg, product_info)

    # Init slow poll
    slow_cache: dict = {}
    slow_cache.update(product_info)
    last_slow_poll  = 0.0
    rtc_synced_date = None
    errors          = 0
    poll            = int(cfg["poll_interval"])

    logging.info("Citire registri slow initiala...")
    slow_data = read_slow_registers(mb, slow_cache)
    slow_data.update(product_info)
    last_slow_poll = time.time()

    logging.info(f"Polling activ (fast={poll}s, slow={SLOW_POLL_INTERVAL}s)...")

    while True:
        t0  = time.time()
        now = datetime.now()

        today = now.date()
        if (now.hour == RTC_CHECK_HOUR and now.minute == RTC_CHECK_MINUTE
                and rtc_synced_date != today):
            rtc_synced_date = today
            mb.check_and_sync_rtc(RTC_DRIFT_THRESH)

        if time.time() - last_slow_poll >= SLOW_POLL_INTERVAL:
            slow_data = read_slow_registers(mb, slow_cache)
            slow_data.update(product_info)
            last_slow_poll = time.time()

        try:
            data = read_all(mb)

            if data is None:
                errors += 1
                logging.warning(f"Citire esuata ({errors}/5)")
                if errors >= 5:
                    logging.error("5 erori -> reconectare...")
                    mb.disconnect()
                    time.sleep(5)
                    mb.connect()
                    errors = 0
            else:
                errors = 0
                data.update(slow_data)
                if connected:
                    publish_state(mq, cfg["mqtt_topic_prefix"], data)
                    logging.info(
                        f"SOC={data.get('battery_soc')}% "
                        f"Vbat={data.get('battery_voltage')}V "
                        f"Ibat={data.get('battery_current')}A "
                        f"Ppv={data.get('pv_power')}W "
                        f"Pac={data.get('ac_active_power')}W "
                        f"Tdc={data.get('temp_dc_side')}C "
                        f"Step={data.get('battery_charge_step')} "
                        f"BatAzi={data.get('battery_charge_today_ah')}Ah "
                        f"BatTot={data.get('battery_charge_total_ah')}Ah"
                    )
                else:
                    logging.warning("MQTT neconectat")

        except Exception as e:
            logging.exception(f"Eroare: {e}")
            errors += 1

        time.sleep(max(0, poll - (time.time() - t0)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Oprire.")
        sys.exit(0)
