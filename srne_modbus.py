#!/usr/bin/env python3
"""
srne_modbus.py - SRNE Invertor Modbus RTU -> MQTT -> Home Assistant
Dispozitiv: Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H
Interfata:  USB-B -> CH341 -> /dev/ttyUSB*
Protocol:   Modbus RTU, addr=1, FC03 read, FC16 write, 9600 8N1

Register map verificat empiric (iPower.net) + doc oficial SRNE v3.9:
  Bloc 0x0100: SOC, Vbat, Ibat, Vpv, Ipv, Ppv, charge state, fault
  Bloc 0x0204: machine state, RTC, AC output, temperaturi (invertor-specific)
  Bloc 0xF02F: energie PV si sarcina (azi + total)
  E-registri:  0xE004 machine state, 0xE204 fault

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import json
import logging
import os
import struct
import sys
import time
from datetime import datetime

import paho.mqtt.client as mqtt
import serial

# ─── DEFAULTS ─────────────────────────────────────────────────────────────────
DEFAULTS = {
    "serial_port":        "/dev/ttyUSB1",
    "modbus_address":     1,
    "poll_interval":      30,
    "mqtt_host":          "core-mosquitto",
    "mqtt_port":          1883,
    "mqtt_user":          "mqtt_local",
    "mqtt_password":      "mqtt2026vidra",
    "mqtt_topic_prefix":  "srne",
    "ha_discovery_prefix":"homeassistant",
    "log_level":          "INFO",
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

# ─── Modbus RTU ───────────────────────────────────────────────────────────────

class ModbusRTU:
    def __init__(self, port, baudrate=9600, timeout=1.0, device_addr=1):
        self.port = port
        self.device_addr = device_addr
        self._baudrate = baudrate
        self._timeout = timeout
        self._ser = None

    def connect(self):
        self._ser = serial.Serial(self.port, self._baudrate, bytesize=8,
                                  parity=serial.PARITY_NONE, stopbits=1,
                                  timeout=self._timeout)
        logging.info(f"Serial OK: {self.port} @ {self._baudrate} bps")

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _transact(self, request, expected_bytes):
        for attempt in range(3):
            try:
                self._ser.reset_input_buffer()
                self._ser.write(request)
                resp = self._ser.read(expected_bytes)
                if len(resp) < expected_bytes:
                    time.sleep(0.1)
                    continue
                crc_recv = struct.unpack("<H", resp[-2:])[0]
                if crc_recv != _crc16(resp[:-2]):
                    logging.warning(f"CRC greșit (attempt {attempt+1})")
                    time.sleep(0.1)
                    continue
                if resp[1] & 0x80:
                    logging.warning(f"Modbus exception: code={resp[2] if len(resp)>2 else '?'}")
                    return None
                return resp
            except serial.SerialException as e:
                logging.error(f"Serial error: {e}")
                time.sleep(0.5)
        return None

    def read_registers(self, reg_start, count):
        resp = self._transact(_build_fc03(self.device_addr, reg_start, count),
                              3 + count * 2 + 2)
        if resp is None:
            return None
        return [struct.unpack(">H", resp[3+i*2:5+i*2])[0] for i in range(count)]

    def write_register(self, reg, value):
        return self._transact(_build_fc06(self.device_addr, reg, value), 8) is not None

    def write_registers(self, reg_start, values):
        return self._transact(_build_fc16(self.device_addr, reg_start, values), 8) is not None

    def sync_rtc(self):
        now = datetime.now()
        yy = now.year - 2002
        values = [(yy << 8) | now.month,
                  (now.day << 8) | now.hour,
                  (now.minute << 8) | now.second]
        ok = self.write_registers(0x020C, values)
        logging.info(f"RTC sync: {'OK' if ok else 'FAIL'} → {now.strftime('%Y-%m-%d %H:%M:%S')}")
        return ok

# ─── Parsare registri ─────────────────────────────────────────────────────────

MACHINE_STATE = {0:"Standby",1:"No anomaly",2:"SW startup",3:"Starting",
                 4:"Line mode",5:"Inverter mode",6:"ECO mode",7:"Fault",
                 8:"Shutdown",9:"Running (inverter)"}
CHARGE_STATE  = {0:"Off",1:"Active",2:"MPPT",3:"Equalizing",
                 4:"Boost",5:"Float",6:"Current limit"}

def parse_0100(regs):
    def r(a): return regs[a - 0x0100]
    t = r(0x0103)
    ctrl_t = ((t >> 8) & 0x7F) * (-1 if (t >> 8) & 0x80 else 1)
    bat_t  = (t & 0x7F) * (-1 if t & 0x80 else 1)
    cs = r(0x010C) & 0xFF
    fault = (r(0x0121) << 16) | r(0x0122)
    return {
        "battery_soc":          r(0x0100) & 0xFF,
        "battery_voltage":      round(r(0x0101) * 0.1, 1),
        "battery_current":      round(r(0x0102) * 0.1, 1),
        "temp_controller":      ctrl_t,
        "temp_battery":         bat_t,
        "load_dc_voltage":      round(r(0x0104) * 0.1, 1),
        "load_dc_current":      round(r(0x0105) * 0.01, 2),
        "load_dc_power":        r(0x0106),
        "pv_voltage":           round(r(0x0107) * 0.1, 1),
        "pv_current":           round(r(0x0108) * 0.01, 2),
        "pv_power":             r(0x0109),
        "charge_state_code":    cs,
        "charge_state":         CHARGE_STATE.get(cs, f"?({cs})"),
        "pv_energy_today_wh":   r(0x0113),
        "load_energy_today_wh": r(0x0114),
        "fault_word":           fault,
        "fault_active":         fault != 0,
    }

def parse_0204(regs):
    def r(a):
        i = a - 0x0204
        return regs[i] if 0 <= i < len(regs) else 0
    ms = r(0x0209) & 0xFF
    r0, r1, r2 = r(0x020C), r(0x020D), r(0x020E)
    try:
        rtc = f"{(r0>>8)+2002:04d}-{r0&0xFF:02d}-{r1>>8:02d}T{r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}"
    except Exception:
        rtc = "invalid"
    pap = r(0x021C)
    pac = r(0x021B)
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

def parse_F02F(regs):
    def r(a):
        i = a - 0xF02F
        return regs[i] if 0 <= i < len(regs) else 0
    return {
        "pv_energy_today_kwh":   round(r(0xF02F) * 0.1, 1),
        "load_energy_today_kwh": round(r(0xF030) * 0.1, 1),
        "pv_energy_total_kwh":   round(r(0xF038) * 0.1, 1),
        "load_energy_total_kwh": round(r(0xF03A) * 0.1, 1),
    }

def read_all(mb):
    result = {"timestamp": datetime.now().isoformat()}
    r0100 = mb.read_registers(0x0100, 35)
    if r0100 is None:
        return None
    result.update(parse_0100(r0100))
    time.sleep(0.1)
    r0204 = mb.read_registers(0x0204, 31)
    if r0204:
        result.update(parse_0204(r0204))
    time.sleep(0.1)
    rF02F = mb.read_registers(0xF02F, 13)
    if rF02F:
        result.update(parse_F02F(rF02F))
    time.sleep(0.1)
    rE004 = mb.read_registers(0xE004, 1)
    if rE004:
        result["e004_machine_state"] = rE004[0]
    time.sleep(0.1)
    rE204 = mb.read_registers(0xE204, 1)
    if rE204:
        result["e204_fault"] = rE204[0]
        result["e204_fault_ok"] = (rE204[0] == 0)
    return result

# ─── MQTT + HA Discovery ──────────────────────────────────────────────────────

SENSORS = [
    ("battery_soc",           "%",   "battery",      "SOC Baterie",         "mdi:battery",            None),
    ("battery_voltage",       "V",   "voltage",      "Tensiune Baterie",    "mdi:battery-charging",   None),
    ("battery_current",       "A",   "current",      "Curent Baterie",      "mdi:current-dc",         None),
    ("temp_controller",       "°C",  "temperature",  "Temp Controller",     "mdi:thermometer",        "diagnostic"),
    ("temp_battery",          "°C",  "temperature",  "Temp Baterie",        "mdi:thermometer",        "diagnostic"),
    ("pv_voltage",            "V",   "voltage",      "Tensiune PV",         "mdi:solar-panel",        None),
    ("pv_current",            "A",   "current",      "Curent PV",           "mdi:solar-panel",        None),
    ("pv_power",              "W",   "power",        "Putere PV",           "mdi:solar-power",        None),
    ("pv_energy_today_kwh",   "kWh", "energy",       "Energie PV Azi",      "mdi:solar-power",        None),
    ("pv_energy_total_kwh",   "kWh", "energy",       "Energie PV Total",    "mdi:solar-power",        None),
    ("ac_output_voltage",     "V",   "voltage",      "Tensiune AC Out",     "mdi:power-plug",         None),
    ("ac_output_frequency",   "Hz",  "frequency",    "Frecventa AC Out",    "mdi:sine-wave",          None),
    ("ac_output_current",     "A",   "current",      "Curent AC Out",       "mdi:current-ac",         None),
    ("ac_active_power",       "W",   "power",        "Putere Activa AC",    "mdi:lightning-bolt",     None),
    ("ac_apparent_power",     "VA",  None,           "Putere Aparenta AC",  "mdi:lightning-bolt",     None),
    ("power_factor",          None,  "power_factor", "Factor Putere",       "mdi:angle-acute",        None),
    ("load_ratio",            "%",   None,           "Sarcina %",           "mdi:gauge",              None),
    ("load_energy_today_kwh", "kWh", "energy",       "Consum Sarcina Azi",  "mdi:home-lightning-bolt",None),
    ("load_energy_total_kwh", "kWh", "energy",       "Consum Sarcina Total","mdi:home-lightning-bolt",None),
    ("temp_dc_side",          "°C",  "temperature",  "Temp DC Side",        "mdi:thermometer",        "diagnostic"),
    ("temp_ac_side",          "°C",  "temperature",  "Temp AC Side",        "mdi:thermometer",        "diagnostic"),
    ("temp_transformer",      "°C",  "temperature",  "Temp Trafo",          "mdi:thermometer",        "diagnostic"),
    ("charge_state",          None,  None,           "Stare Incarcare",     "mdi:battery-charging",   "diagnostic"),
    ("machine_state",         None,  None,           "Stare Invertor",      "mdi:information",        "diagnostic"),
    ("rtc_datetime",          None,  "timestamp",    "RTC Invertor",        "mdi:clock",              "diagnostic"),
    ("e204_fault",            None,  None,           "Cod Fault",           "mdi:alert",              "diagnostic"),
]

DEVICE = {
    "identifiers": ["srne_hf2450s80h"],
    "name": "SRNE Invertor HF2450S80H",
    "model": "HF2450S80H (Easun ISI Max II 3.6kW)",
    "manufacturer": "SRNE Solar",
}

def publish_discovery(client, cfg):
    prefix = cfg["ha_discovery_prefix"]
    state  = f"{cfg['mqtt_topic_prefix']}/state"
    for key, unit, dc, name, icon, ent_cat in SENSORS:
        p = {"name": name, "unique_id": f"srne_{key}",
             "state_topic": state, "value_template": f"{{{{ value_json.{key} }}}}",
             "device": DEVICE, "icon": icon}
        if unit:    p["unit_of_measurement"] = unit
        if dc:      p["device_class"] = dc
        if ent_cat: p["entity_category"] = ent_cat
        if unit == "kWh": p["state_class"] = "total_increasing"
        elif unit in ("W","VA","V","A","%","Hz"): p["state_class"] = "measurement"
        client.publish(f"{prefix}/sensor/srne_{key}/config", json.dumps(p), retain=True)
    client.publish(f"{prefix}/binary_sensor/srne_fault_active/config", json.dumps({
        "name": "Fault Activ Invertor", "unique_id": "srne_fault_active",
        "state_topic": state,
        "value_template": "{{ 'ON' if value_json.fault_active else 'OFF' }}",
        "device_class": "problem", "device": DEVICE, "entity_category": "diagnostic",
    }), retain=True)
    logging.info("HA auto-discovery publicat.")

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    cfg = load_config()
    logging.basicConfig(
        level=getattr(logging, cfg.get("log_level","INFO").upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logging.info("=== SRNE Invertor Modbus v1.0.0 ===")
    logging.info(f"Port: {cfg['serial_port']} | Addr: {cfg['modbus_address']} | Poll: {cfg['poll_interval']}s")

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

    mq.on_connect = on_connect
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

    if connected:
        publish_discovery(mq, cfg)
    mb.sync_rtc()

    poll = cfg["poll_interval"]
    errors = 0
    last_rtc = time.time()

    while True:
        t0 = time.time()
        try:
            data = read_all(mb)
            if data is None:
                errors += 1
                if errors >= 5:
                    logging.error("5 erori consecutive, reconectare serial...")
                    mb.disconnect(); time.sleep(5); mb.connect(); errors = 0
            else:
                errors = 0
                if connected:
                    mq.publish(f"{cfg['mqtt_topic_prefix']}/state", json.dumps(data, default=str))
                    logging.info(f"SOC={data.get('battery_soc')}% "
                                 f"Vbat={data.get('battery_voltage')}V "
                                 f"Ppv={data.get('pv_power')}W "
                                 f"Pac={data.get('ac_active_power')}W "
                                 f"State={data.get('machine_state')}")
        except Exception as e:
            logging.exception(f"Loop error: {e}")
            errors += 1

        if time.time() - last_rtc >= 3600:
            mb.sync_rtc()
            last_rtc = time.time()

        time.sleep(max(0, poll - (time.time() - t0)))

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
