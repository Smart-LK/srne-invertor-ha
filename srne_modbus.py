#!/usr/bin/env python3
"""
srne_modbus.py v1.3.1 - SRNE Invertor Modbus RTU -> MQTT -> Home Assistant
===========================================================================
Dispozitiv: Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H
Interfata:  Port USB-B (mufa patrata) -> CH340 -> /dev/ttyUSB*
Protocol:   Modbus RTU, addr=1, FC03 read, FC06/FC16 write, 9600 8N1

Registri confirmati pe firmware HF2450S80H:
  0x0100 x 15: SOC, Vbat, Ibat(signed), Vpv string, Ipv, Ppv, charge step
  0x0204 x 31: machine state, RTC, AC output, load ratio, temperaturi
  0xF02F x 13: energie PV azi/total, consum azi/total
  0xE004 x 1:  machine state (mai detaliat decat 0x0209)
  0xE204 x 1:  fault/alarm
  0xE20F x 1:  output priority (confirmat: 3=Solar only/Off-grid)

Bloc 0xE000-0xE012 (confirmat empiric pe HF2450S80H):
  E001 = 50   -> x10 = 500V  Vpv max string
  E002 = 100  -> x1  = 100A  I max incarcare total (PV + retea)
  E003 = 24   -> x1  =  24V  tensiune nominala baterie
  E005-E00E   -> 10 valori curba (Peukert/SOC lookup) - nume TBD utilizator
  E012 = 365  -> x10 = 3650W = 3.6kW putere nominala
  E010/E011   -> nu contin versiunile firmware APP/Boot

Bloc 0xF000-0xF00F (confirmat empiric pe HF2450S80H):
  16 valori x0.1 kWh = istoricul productiei PV ultimele 16 zile
  F000 = azi (confirmat: 32=3.2kWh=PV azi), F001=ieri, ..., F00F=ziua-15

HA Energy Dashboard:
  pv_energy_total_kwh + load_energy_total_kwh (total_increasing)

Changelog:
  v1.3.1 - fix OUTPUT_PRIORITY: adaugat 3=Solar only/Off-grid (confirmat empiric)
           fix model info: E001=Vpv_max(x10), E002=Ichg_max, E003=Vnom_bat
           adaugat E012=putere nominala (365x10=3650W=3.6kW)
           citire E000 x 19 (acopera E000-E012 intr-o cerere)
           adaugat E005-E00E: curba Peukert/SOC (10 valori, logat+publicat ca lista)
           adaugat F000-F00F: istoricul PV 16 zile (logat + in JSON state)
           eliminat fw_app/boot_version (E010/E011 nu contin versiunile)
           senzori HA noi: model_rated_power_w, model_pv_max_voltage_v,
             model_max_chg_current_a, model_bat_nominal_v
  v1.3.0 - RTC smart sync la 00:05, drift check, output_priority, slow poll
  v1.2.0 - eliminat temp_controller/temp_battery, precision display
  v1.1.0 - recv_slave_frame filtrare bus, 0x0100x15, Ibat signed, log fisier
  v1.0.0 - versiune initiala

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

# --- Config -------------------------------------------------------------------

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

# --- Logging dual: consola + fisier -------------------------------------------

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

# --- CRC16 Modbus -------------------------------------------------------------

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

def _to_signed16(val: int) -> int:
    return val if val < 0x8000 else val - 0x10000

# --- Receive cu filtrare dupa adresa slave ------------------------------------

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
                byte_count = rest[2]
                if byte_count != expected_regs * 2:
                    i += 1
                    continue
                total = 3 + byte_count + 2
                if len(rest) < total:
                    break
                frame = bytes(rest[:total])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0:
                        logging.debug(f"Bus: ignorat {ignored}b de la alte dispozitive")
                    return frame, False
                i += 1
                continue
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0:
                        logging.debug(f"Bus: ignorat {ignored}b de la alte dispozitive")
                    return frame, True
                i += 1
                continue
            i += 1

    if ignored > 0:
        logging.debug(f"Bus: ignorat {ignored}b (timeout)")
    return None, False

# --- Modbus RTU ---------------------------------------------------------------

class ModbusRTU:
    def __init__(self, port, baudrate=9600, device_addr=1):
        self.port        = port
        self.device_addr = device_addr
        self._baudrate   = baudrate
        self._ser        = None

    def connect(self):
        self._ser = serial.Serial(
            self.port, self._baudrate,
            bytesize=8, parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)
        logging.info(f"Serial OK: {self.port} @ {self._baudrate} bps")

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_registers(self, reg_start: int, count: int) -> list | None:
        request = _build_fc03(self.device_addr, reg_start, count)
        self._ser.reset_input_buffer()
        time.sleep(0.05)
        self._ser.write(request)
        frame, is_exc = _recv_slave_frame(self._ser, self.device_addr, count)
        if frame is None:
            logging.warning(f"Timeout FC03 0x{reg_start:04X}x{count}")
            return None
        if is_exc:
            logging.warning(f"Exception FC03 0x{reg_start:04X}: code=0x{frame[2]:02X}")
            return None
        return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(count)]

    def write_register(self, reg: int, value: int) -> bool:
        request = _build_fc06(self.device_addr, reg, value)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        resp = self._ser.read(8)
        if len(resp) == 8 and struct.unpack("<H", resp[-2:])[0] == _crc16(resp[:-2]):
            return True
        logging.warning(f"FC06 0x{reg:04X} = {value}: raspuns invalid")
        return False

    def write_registers(self, reg_start: int, values: list) -> bool:
        request = _build_fc16(self.device_addr, reg_start, values)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        resp = self._ser.read(8)
        if len(resp) == 8 and struct.unpack("<H", resp[-2:])[0] == _crc16(resp[:-2]):
            return True
        logging.warning(f"FC16 0x{reg_start:04X} x{len(values)}: raspuns invalid")
        return False

    def read_rtc(self) -> datetime | None:
        regs = self.read_registers(0x020C, 3)
        if regs is None:
            return None
        try:
            r0, r1, r2 = regs
            return datetime((r0 >> 8) + 2002, r0 & 0xFF,
                            r1 >> 8, r1 & 0xFF, r2 >> 8, r2 & 0xFF)
        except Exception as e:
            logging.warning(f"RTC parse eroare: {e}")
            return None

    def sync_rtc(self) -> bool:
        now = datetime.now()
        yy  = now.year - 2002
        values = [(yy << 8) | now.month,
                  (now.day << 8) | now.hour,
                  (now.minute << 8) | now.second]
        ok = self.write_registers(0x020C, values)
        logging.info(f"RTC sync: {'OK' if ok else 'FAIL'} -> {now.strftime('%Y-%m-%d %H:%M:%S')}")
        return ok

    def check_and_sync_rtc(self, drift_threshold_s: int = 60) -> bool:
        invertor_time = self.read_rtc()
        now = datetime.now()
        if invertor_time is None:
            logging.warning("RTC check: nu pot citi ora invertorului")
            return False
        drift_s = abs((now - invertor_time).total_seconds())
        logging.info(f"RTC check: invertor={invertor_time.strftime('%H:%M:%S')} "
                     f"sistem={now.strftime('%H:%M:%S')} drift={drift_s:.0f}s")
        if drift_s > drift_threshold_s:
            logging.info(f"RTC drift {drift_s:.0f}s > {drift_threshold_s}s -- sincronizare...")
            return self.sync_rtc()
        else:
            logging.info(f"RTC drift {drift_s:.0f}s in limite -- fara sync")
            return False

# --- Parsare registri ---------------------------------------------------------

MACHINE_STATE = {
    0: "Standby", 1: "No anomaly", 2: "SW startup", 3: "Starting",
    4: "Line mode", 5: "Inverter mode", 6: "ECO mode",
    7: "Fault", 8: "Shutdown", 9: "Running (inverter)"
}

CHARGE_STEP = {
    0: "Off", 1: "Const current", 2: "MPPT", 3: "Equalize",
    4: "Boost", 5: "Float", 6: "Current limit", 7: "Const voltage",
}

# Confirmat empiric: 3 = Solar only / Off-grid
OUTPUT_PRIORITY = {
    0: "Utility first", 1: "Solar first", 2: "SBU priority",
    3: "Solar only / Off-grid",
}


def parse_0100(regs: list) -> dict:
    def r(a):
        i = a - 0x0100
        return regs[i] if 0 <= i < len(regs) else 0
    cs   = r(0x010C) & 0xFF
    ibat = _to_signed16(r(0x0102))
    return {
        "battery_soc":         r(0x0100) & 0xFF,
        "battery_voltage":     round(r(0x0101) * 0.1, 1),
        "battery_current":     round(ibat * 0.1, 1),
        "pv_voltage":          round(r(0x0107) * 0.1, 1),
        "pv_current":          round(r(0x0108) * 0.01, 2),
        "pv_power":            r(0x0109),
        "battery_charge_step": CHARGE_STEP.get(cs, f"?({cs})"),
    }


def parse_0204(regs: list) -> dict:
    def r(a):
        i = a - 0x0204
        return regs[i] if 0 <= i < len(regs) else 0
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
        "load_ratio":          r(0x0210),
        "running_seconds":     r(0x0212),
        "ac_output_voltage":   round(r(0x0216) * 0.1, 1),
        "ac_output_frequency": round(r(0x0218) * 0.01, 2),
        "ac_output_current":   round(r(0x0219) * 0.1, 1),
        "ac_active_power":     pac,
        "ac_apparent_power":   pap,
        "power_factor":        round(pac / pap, 3) if pap else 0.0,
        "temp_dc_side":        round(r(0x0220) * 0.1, 1),
        "temp_ac_side":        round(r(0x0221) * 0.1, 1),
        "temp_transformer":    round(r(0x0222) * 0.1, 1),
    }


def parse_F02F(regs: list) -> dict:
    def r(a):
        i = a - 0xF02F
        return regs[i] if 0 <= i < len(regs) else 0
    return {
        "pv_energy_today_kwh":   round(r(0xF02F) * 0.1, 1),
        "load_energy_today_kwh": round(r(0xF030) * 0.1, 1),
        "pv_energy_total_kwh":   round(r(0xF038) * 0.1, 1),
        "load_energy_total_kwh": round(r(0xF03A) * 0.1, 1),
    }


def parse_E000_block(regs: list) -> dict:
    """
    Bloc E000 x 19 regs (E000-E012).
    E001: x10 = 500V Vpv max
    E002: x1  = 100A I max incarcare total
    E003: x1  = 24V  tensiune nominala baterie
    E005-E00E: 10 valori curba Peukert/SOC (nume TBD de utilizator)
    E012: x10 = 3650W = 3.6kW putere nominala
    """
    def r(i): return regs[i] if i < len(regs) else 0
    peukert = [r(i) for i in range(5, 15)]  # E005-E00E (offset 5..14 din E000)
    return {
        "model_pv_max_voltage_v":  r(1) * 10,    # E001
        "model_max_chg_current_a": r(2),          # E002
        "model_bat_nominal_v":     r(3),          # E003
        "model_rated_power_w":     r(18) * 10,   # E012 (offset 18 = 0x12)
        "peukert_curve":           peukert,       # E005-E00E, 10 valori, nume TBD
    }


def parse_F000_block(regs: list) -> dict:
    """
    Bloc F000 x 16 regs = istoricul productiei PV ultimele 16 zile.
    F000=azi (confirmat: 32=3.2kWh), F001=ieri, ..., F00F=ziua-15.
    Scale: x0.1 kWh.
    In JSON state ca pv_hist_day_0..15.
    """
    result = {}
    for i, v in enumerate(regs[:16]):
        result[f"pv_hist_day_{i}"] = round(v * 0.1, 1)
    return result


def read_slow_registers(mb: ModbusRTU, state_cache: dict) -> dict:
    """Registri cititi o data pe ora. Sarite silentios daca returneaza exception."""
    result = {}

    # Output priority (0xE20F)
    try:
        r = mb.read_registers(0xE20F, 1)
        if r is not None:
            op  = r[0] & 0xFF
            val = OUTPUT_PRIORITY.get(op, f"?({op})")
            result["output_priority"] = val
            if state_cache.get("output_priority") != val:
                logging.info(f"Output priority: {val} (raw: {op})")
            state_cache["output_priority"] = val
        elif "output_priority" in state_cache:
            result["output_priority"] = state_cache["output_priority"]
        time.sleep(0.15)
    except Exception:
        if "output_priority" in state_cache:
            result["output_priority"] = state_cache["output_priority"]

    # Model info + curba Peukert: E000 x 19 (E000-E012)
    model_keys = ("model_rated_power_w", "model_pv_max_voltage_v",
                  "model_max_chg_current_a", "model_bat_nominal_v", "peukert_curve")
    try:
        r = mb.read_registers(0xE000, 19)
        if r is not None:
            parsed = parse_E000_block(r)
            result.update(parsed)
            if "model_rated_power_w" not in state_cache:
                logging.info(
                    f"Model: Pnom={parsed['model_rated_power_w']}W "
                    f"Vpv_max={parsed['model_pv_max_voltage_v']}V "
                    f"Ichg_max={parsed['model_max_chg_current_a']}A "
                    f"Vbat_nom={parsed['model_bat_nominal_v']}V"
                )
                logging.info(f"Peukert/SOC curve E005-E00E: {parsed['peukert_curve']}")
            state_cache.update(parsed)
        else:
            for k in model_keys:
                if k in state_cache:
                    result[k] = state_cache[k]
        time.sleep(0.15)
    except Exception:
        for k in model_keys:
            if k in state_cache:
                result[k] = state_cache[k]

    # Istoricul PV 16 zile: F000 x 16
    hist_keys = [f"pv_hist_day_{i}" for i in range(16)]
    try:
        r = mb.read_registers(0xF000, 16)
        if r is not None:
            hist = parse_F000_block(r)
            result.update(hist)
            if "pv_hist_day_0" not in state_cache:
                logging.info(f"Istoric PV 16 zile (kWh): {[round(v*0.1,1) for v in r]}")
            state_cache.update(hist)
        else:
            for k in hist_keys:
                if k in state_cache:
                    result[k] = state_cache[k]
        time.sleep(0.15)
    except Exception:
        for k in hist_keys:
            if k in state_cache:
                result[k] = state_cache[k]

    return result


def read_all(mb: ModbusRTU) -> dict | None:
    result = {"timestamp": datetime.now().isoformat()}

    r0100 = mb.read_registers(0x0100, 15)
    if r0100 is None:
        return None
    result.update(parse_0100(r0100))
    time.sleep(0.15)

    r0204 = mb.read_registers(0x0204, 31)
    if r0204:
        result.update(parse_0204(r0204))
    time.sleep(0.15)

    rF02F = mb.read_registers(0xF02F, 13)
    if rF02F:
        result.update(parse_F02F(rF02F))
    time.sleep(0.15)

    rE004 = mb.read_registers(0xE004, 1)
    if rE004:
        result["e004_machine_state"] = rE004[0]
    time.sleep(0.1)

    rE204 = mb.read_registers(0xE204, 1)
    if rE204:
        result["e204_fault"]   = rE204[0]
        result["fault_active"] = (rE204[0] != 0)
    else:
        result["fault_active"] = False

    return result

# --- MQTT + HA Auto-Discovery -------------------------------------------------

SENSORS = [
    # key                       unit   dc              name                       icon                      ent_cat      prec
    ("battery_soc",            "%",   "battery",      "SOC Baterie",             "mdi:battery",            None,        0),
    ("battery_voltage",        "V",   "voltage",      "Tensiune Baterie",        "mdi:battery-charging",   None,        1),
    ("battery_current",        "A",   "current",      "Curent Baterie",          "mdi:current-dc",         None,        1),
    ("pv_voltage",             "V",   "voltage",      "Tensiune PV",             "mdi:solar-panel",        None,        1),
    ("pv_current",             "A",   "current",      "Curent PV",               "mdi:solar-panel",        None,        2),
    ("pv_power",               "W",   "power",        "Putere PV",               "mdi:solar-power",        None,        0),
    ("pv_energy_today_kwh",    "kWh", "energy",       "Energie PV Azi",          "mdi:solar-power",        None,        1),
    ("pv_energy_total_kwh",    "kWh", "energy",       "Energie PV Total",        "mdi:solar-power",        None,        1),
    ("ac_output_voltage",      "V",   "voltage",      "Tensiune AC Out",         "mdi:power-plug",         None,        1),
    ("ac_output_frequency",    "Hz",  "frequency",    "Frecventa AC Out",        "mdi:sine-wave",          None,        2),
    ("ac_output_current",      "A",   "current",      "Curent AC Out",           "mdi:current-ac",         None,        1),
    ("ac_active_power",        "W",   "power",        "Putere Activa AC",        "mdi:lightning-bolt",     None,        0),
    ("ac_apparent_power",      "VA",  None,           "Putere Aparenta AC",      "mdi:lightning-bolt",     None,        0),
    ("power_factor",           None,  "power_factor", "Factor Putere",           "mdi:angle-acute",        None,        3),
    ("load_ratio",             "%",   None,           "Sarcina %",               "mdi:gauge",              None,        0),
    ("load_energy_today_kwh",  "kWh", "energy",       "Consum Sarcina Azi",      "mdi:home-lightning-bolt",None,        1),
    ("load_energy_total_kwh",  "kWh", "energy",       "Consum Sarcina Total",    "mdi:home-lightning-bolt",None,        1),
    ("temp_dc_side",           "°C",  "temperature",  "Temp DC Side",            "mdi:thermometer",        "diagnostic",1),
    ("temp_ac_side",           "°C",  "temperature",  "Temp AC Side",            "mdi:thermometer",        "diagnostic",1),
    ("temp_transformer",       "°C",  "temperature",  "Temp Trafo",              "mdi:thermometer",        "diagnostic",1),
    ("battery_charge_step",    None,  None,           "Etapa Incarcare",         "mdi:battery-charging",   "diagnostic",None),
    ("machine_state",          None,  None,           "Stare Invertor",          "mdi:information",        "diagnostic",None),
    ("output_priority",        None,  None,           "Prioritate Iesire",       "mdi:priority-high",      "diagnostic",None),
    ("e204_fault",             None,  None,           "Cod Fault",               "mdi:alert",              "diagnostic",None),
    # Parametri model (statici, cititi 1/ora)
    ("model_rated_power_w",    "W",   None,           "Putere Nominala",         "mdi:flash",              "diagnostic",0),
    ("model_pv_max_voltage_v", "V",   None,           "Vpv Max String",          "mdi:solar-panel-large",  "diagnostic",0),
    ("model_max_chg_current_a","A",   None,           "I Max Incarcare Total",   "mdi:current-dc",         "diagnostic",0),
    ("model_bat_nominal_v",    "V",   None,           "Tensiune Nominala Bat",   "mdi:battery",            "diagnostic",0),
]

TOTAL_INCREASING_KEYS   = {"pv_energy_total_kwh", "load_energy_total_kwh"}
MEASUREMENT_ENERGY_KEYS = {"pv_energy_today_kwh", "load_energy_today_kwh"}
NO_STATE_CLASS_KEYS     = {"model_rated_power_w", "model_pv_max_voltage_v",
                           "model_max_chg_current_a", "model_bat_nominal_v"}

DEVICE = {
    "identifiers":  ["srne_hf2450s80h"],
    "name":         "SRNE Invertor HF2450S80H",
    "model":        "HF2450S80H (Easun ISI Max II 3.6kW/24V)",
    "manufacturer": "SRNE Solar",
    "sw_version":   "Modbus RTU v1.3.1",
}


def publish_discovery(client, cfg: dict):
    prefix = cfg["ha_discovery_prefix"]
    state  = f"{cfg['mqtt_topic_prefix']}/state"

    for key, unit, dc, name, icon, ent_cat, precision in SENSORS:
        p = {
            "name":           name,
            "unique_id":      f"srne_{key}",
            "state_topic":    state,
            "value_template": f"{{{{ value_json.{key} }}}}",
            "device":         DEVICE,
            "icon":           icon,
        }
        if unit:      p["unit_of_measurement"] = unit
        if dc:        p["device_class"] = dc
        if ent_cat:   p["entity_category"] = ent_cat
        if precision is not None:
            p["suggested_display_precision"] = precision

        if key in TOTAL_INCREASING_KEYS:
            p["state_class"] = "total_increasing"
        elif key in MEASUREMENT_ENERGY_KEYS:
            p["state_class"] = "measurement"
        elif key not in NO_STATE_CLASS_KEYS and unit in ("W","VA","V","A","%","Hz","°C"):
            p["state_class"] = "measurement"

        client.publish(f"{prefix}/sensor/srne_{key}/config",
                       json.dumps(p), retain=True)

    client.publish(f"{prefix}/binary_sensor/srne_fault_active/config", json.dumps({
        "name":            "Fault Activ Invertor",
        "unique_id":       "srne_fault_active",
        "state_topic":     state,
        "value_template":  "{{ 'ON' if value_json.fault_active else 'OFF' }}",
        "device_class":    "problem",
        "device":          DEVICE,
        "entity_category": "diagnostic",
    }), retain=True)

    logging.info("HA auto-discovery publicat.")


def publish_state(client, topic_prefix: str, data: dict):
    out = dict(data)
    # Serializam lista peukert_curve ca string JSON pentru compatibilitate MQTT
    if "peukert_curve" in out and isinstance(out["peukert_curve"], list):
        out["peukert_curve"] = json.dumps(out["peukert_curve"])
    client.publish(f"{topic_prefix}/state",
                   json.dumps(out, default=str), retain=False)

# --- Main ---------------------------------------------------------------------

SLOW_POLL_INTERVAL = 3600
RTC_CHECK_HOUR     = 0
RTC_CHECK_MINUTE   = 5
RTC_DRIFT_THRESH   = 60

def main():
    cfg = load_config()
    setup_logging(cfg.get("log_level", "INFO"))

    logging.info("=" * 55)
    logging.info("  SRNE Invertor Modbus v1.3.1")
    logging.info(f"  Log: {LOG_FILE}")
    logging.info(f"  Port: {cfg['serial_port']} | Addr: {cfg['modbus_address']} | Poll: {cfg['poll_interval']}s")
    logging.info(f"  RTC check: zilnic la {RTC_CHECK_HOUR:02d}:{RTC_CHECK_MINUTE:02d}, sync daca drift > {RTC_DRIFT_THRESH}s")
    logging.info("=" * 55)

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
            logging.error(f"MQTT connect: {e}. Retry 10s...")
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

    if connected:
        publish_discovery(mq, cfg)

    slow_cache: dict = {}
    last_slow_poll   = 0.0
    rtc_synced_date  = None
    errors           = 0
    poll             = int(cfg["poll_interval"])

    logging.info("Citire registri slow (model info, Peukert, istoric PV, output priority)...")
    slow_data = read_slow_registers(mb, slow_cache)
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
            last_slow_poll = time.time()

        try:
            data = read_all(mb)

            if data is None:
                errors += 1
                logging.warning(f"Citire esuata ({errors}/5)")
                if errors >= 5:
                    logging.error("5 erori consecutive -- reconectare serial...")
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
                        f"State={data.get('machine_state')}"
                    )
                else:
                    logging.warning("MQTT neconectat -- date necomunicate")

        except Exception as e:
            logging.exception(f"Eroare neasteptata: {e}")
            errors += 1

        time.sleep(max(0, poll - (time.time() - t0)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Oprire la cerere.")
        sys.exit(0)
