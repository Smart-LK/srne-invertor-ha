#!/usr/bin/env python3
"""
srne_modbus.py v1.1.0 - SRNE Invertor Modbus RTU -> MQTT -> Home Assistant
===========================================================================
Dispozitiv: Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H
Interfata:  Port USB-B (mufa patrata) -> CH340 -> /dev/ttyUSB*
Protocol:   Modbus RTU, addr=1, FC03 read, FC06/FC16 write, 9600 8N1

Registri confirmati pe firmware HF2450S80H:
  0x0100 x 15: SOC, Vbat, Ibat(signed), temps, Vpv string, Ipv, Ppv, charge step
  0x0204 x 31: machine state, RTC, AC output, load ratio, temperaturi
  0xF02F x 13: energie PV azi/total, consum azi/total
  0xE004 x 1:  machine state
  0xE204 x 1:  fault/alarm

Changelog:
  v1.1.0 - recv_slave_frame() cu filtrare dupa adresa slave (fix CRC errors)
           citire 0x0100 x 15 regs (fix exception 0x0A la 35 regs)
           Ibat interpretat ca signed int16
           eliminat registri inexistenti (0x0113, 0x0114, 0x0121, 0x0122)
           logging dual: consola + fisier srne_modbus.log
  v1.0.0 - versiune initiala

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

# ─── Logging dual: consola + fisier ───────────────────────────────────────────

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

def _to_signed16(val: int) -> int:
    """uint16 → int16 cu semn. Folosit pentru Ibat (negativ=incarcare)."""
    return val if val < 0x8000 else val - 0x10000

# ─── Receive cu filtrare dupa adresa slave ────────────────────────────────────

def _recv_slave_frame(ser, slave_addr: int, expected_regs: int, timeout=3.0):
    """
    Citeste bytes de pe bus si cauta un frame Modbus valid pentru slave_addr.
    Bytes cu alta adresa (ex. trafic intern invertor->BMS) sunt ignorati.

    Frame normal:    [slave_addr][0x03][byte_count][data...][CRC_L][CRC_H]
    Frame exception: [slave_addr][0x83][exc_code][CRC_L][CRC_H]

    Returneaza (frame_bytes, is_exception) sau (None, False) la timeout.
    """
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

            # Frame normal FC03
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
                        logging.debug(f"Bus: ignorat {ignored}b trafic de la alte dispozitive")
                    return frame, False
                i += 1
                continue

            # Frame exception FC03 (0x83)
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0:
                        logging.debug(f"Bus: ignorat {ignored}b trafic de la alte dispozitive")
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
        self._ser = serial.Serial(
            self.port, self._baudrate,
            bytesize=8, parity=serial.PARITY_NONE, stopbits=1,
            timeout=0.1   # timeout scurt — recv_slave_frame gestioneaza timeout-ul
        )
        logging.info(f"Serial OK: {self.port} @ {self._baudrate} bps")

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_registers(self, reg_start: int, count: int) -> list | None:
        """FC03: citeste count registri de la reg_start cu filtrare bus."""
        request = _build_fc03(self.device_addr, reg_start, count)
        self._ser.reset_input_buffer()
        time.sleep(0.05)
        self._ser.write(request)

        frame, is_exc = _recv_slave_frame(self._ser, self.device_addr, count)

        if frame is None:
            logging.warning(f"Timeout FC03 0x{reg_start:04X}x{count}")
            return None

        if is_exc:
            exc_code = frame[2]
            logging.warning(f"Exception FC03 0x{reg_start:04X}: code=0x{exc_code:02X}")
            return None

        return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(count)]

    def write_register(self, reg: int, value: int) -> bool:
        """FC06: scrie un singur registru."""
        request = _build_fc06(self.device_addr, reg, value)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        resp = self._ser.read(8)
        if len(resp) == 8 and struct.unpack("<H", resp[-2:])[0] == _crc16(resp[:-2]):
            return True
        logging.warning(f"FC06 0x{reg:04X} = {value}: raspuns invalid")
        return False

    def write_registers(self, reg_start: int, values: list) -> bool:
        """FC16: scrie mai multi registri consecutivi."""
        request = _build_fc16(self.device_addr, reg_start, values)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        resp = self._ser.read(8)
        if len(resp) == 8 and struct.unpack("<H", resp[-2:])[0] == _crc16(resp[:-2]):
            return True
        logging.warning(f"FC16 0x{reg_start:04X} x{len(values)}: raspuns invalid")
        return False

    def sync_rtc(self) -> bool:
        """Sincronizeaza ceasul invertorului cu ora sistemului."""
        now = datetime.now()
        yy  = now.year - 2002
        values = [(yy << 8) | now.month,
                  (now.day << 8) | now.hour,
                  (now.minute << 8) | now.second]
        ok = self.write_registers(0x020C, values)
        logging.info(f"RTC sync: {'OK' if ok else 'FAIL'} → {now.strftime('%Y-%m-%d %H:%M:%S')}")
        return ok

# ─── Parsare registri ─────────────────────────────────────────────────────────

MACHINE_STATE = {
    0: "Standby", 1: "No anomaly", 2: "SW startup", 3: "Starting",
    4: "Line mode", 5: "Inverter mode", 6: "ECO mode",
    7: "Fault", 8: "Shutdown", 9: "Running (inverter)"
}
CHARGE_STATE = {
    0: "Off", 1: "Active", 2: "MPPT", 3: "Equalizing",
    4: "Boost", 5: "Float", 6: "Current limit"
}


def parse_0100(regs: list) -> dict:
    """
    Bloc 0x0100 x 15 regs (0x0100-0x010E).
    Ibat: signed int16 — negativ=incarcare baterie, pozitiv=descarcare.
    Vpv: tensiunea stringului PV (ex. 382V pentru 10 panouri serie).
    """
    def r(a):
        i = a - 0x0100
        return regs[i] if 0 <= i < len(regs) else 0

    t     = r(0x0103)
    ctrl_t = ((t >> 8) & 0x7F) * (-1 if (t >> 8) & 0x80 else 1)
    bat_t  = (t & 0x7F) * (-1 if t & 0x80 else 1)
    cs    = r(0x010C) & 0xFF
    ibat  = _to_signed16(r(0x0102))

    return {
        "battery_soc":       r(0x0100) & 0xFF,
        "battery_voltage":   round(r(0x0101) * 0.1, 1),
        "battery_current":   round(ibat * 0.1, 1),   # signed: neg=incarcare
        "temp_controller":   ctrl_t,
        "temp_battery":      bat_t,
        "pv_voltage":        round(r(0x0107) * 0.1, 1),
        "pv_current":        round(r(0x0108) * 0.01, 2),
        "pv_power":          r(0x0109),
        "charge_state_code": cs,
        "charge_state":      CHARGE_STATE.get(cs, f"?({cs})"),
    }


def parse_0204(regs: list) -> dict:
    """Bloc 0x0204 x 31 regs — AC output, RTC, temperaturi (invertor-specific)."""
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
    """Bloc 0xF02F x 13 regs — energie zilnica si cumulativa."""
    def r(a):
        i = a - 0xF02F
        return regs[i] if 0 <= i < len(regs) else 0

    return {
        "pv_energy_today_kwh":   round(r(0xF02F) * 0.1, 1),
        "load_energy_today_kwh": round(r(0xF030) * 0.1, 1),
        "pv_energy_total_kwh":   round(r(0xF038) * 0.1, 1),
        "load_energy_total_kwh": round(r(0xF03A) * 0.1, 1),
    }


def read_all(mb: ModbusRTU) -> dict | None:
    """Citeste toate datele de la invertor. Returneaza None la eroare critica."""
    result = {"timestamp": datetime.now().isoformat()}

    # Bloc 0x0100 x 15 (nu 35 — firmware-ul returneaza exception la 35)
    r0100 = mb.read_registers(0x0100, 15)
    if r0100 is None:
        return None
    result.update(parse_0100(r0100))
    time.sleep(0.15)

    # Bloc 0x0204 x 31 — AC output + RTC + temperaturi
    r0204 = mb.read_registers(0x0204, 31)
    if r0204:
        result.update(parse_0204(r0204))
    time.sleep(0.15)

    # Bloc 0xF02F x 13 — energie
    rF02F = mb.read_registers(0xF02F, 13)
    if rF02F:
        result.update(parse_F02F(rF02F))
    time.sleep(0.15)

    # E004 — machine state
    rE004 = mb.read_registers(0xE004, 1)
    if rE004:
        result["e004_machine_state"] = rE004[0]
    time.sleep(0.1)

    # E204 — fault/alarm
    rE204 = mb.read_registers(0xE204, 1)
    if rE204:
        result["e204_fault"]    = rE204[0]
        result["fault_active"]  = (rE204[0] != 0)
    else:
        result["fault_active"] = False

    return result

# ─── MQTT + HA Auto-Discovery ─────────────────────────────────────────────────

SENSORS = [
    # (key, unit, device_class, name, icon, entity_category)
    ("battery_soc",           "%",   "battery",      "SOC Baterie",          "mdi:battery",            None),
    ("battery_voltage",       "V",   "voltage",      "Tensiune Baterie",     "mdi:battery-charging",   None),
    ("battery_current",       "A",   "current",      "Curent Baterie",       "mdi:current-dc",         None),
    ("temp_controller",       "°C",  "temperature",  "Temp Controller",      "mdi:thermometer",        "diagnostic"),
    ("temp_battery",          "°C",  "temperature",  "Temp Baterie",         "mdi:thermometer",        "diagnostic"),
    ("pv_voltage",            "V",   "voltage",      "Tensiune PV",          "mdi:solar-panel",        None),
    ("pv_current",            "A",   "current",      "Curent PV",            "mdi:solar-panel",        None),
    ("pv_power",              "W",   "power",        "Putere PV",            "mdi:solar-power",        None),
    ("pv_energy_today_kwh",   "kWh", "energy",       "Energie PV Azi",       "mdi:solar-power",        None),
    ("pv_energy_total_kwh",   "kWh", "energy",       "Energie PV Total",     "mdi:solar-power",        None),
    ("ac_output_voltage",     "V",   "voltage",      "Tensiune AC Out",      "mdi:power-plug",         None),
    ("ac_output_frequency",   "Hz",  "frequency",    "Frecventa AC Out",     "mdi:sine-wave",          None),
    ("ac_output_current",     "A",   "current",      "Curent AC Out",        "mdi:current-ac",         None),
    ("ac_active_power",       "W",   "power",        "Putere Activa AC",     "mdi:lightning-bolt",     None),
    ("ac_apparent_power",     "VA",  None,           "Putere Aparenta AC",   "mdi:lightning-bolt",     None),
    ("power_factor",          None,  "power_factor", "Factor Putere",        "mdi:angle-acute",        None),
    ("load_ratio",            "%",   None,           "Sarcina %",            "mdi:gauge",              None),
    ("load_energy_today_kwh", "kWh", "energy",       "Consum Sarcina Azi",   "mdi:home-lightning-bolt",None),
    ("load_energy_total_kwh", "kWh", "energy",       "Consum Sarcina Total", "mdi:home-lightning-bolt",None),
    ("temp_dc_side",          "°C",  "temperature",  "Temp DC Side",         "mdi:thermometer",        "diagnostic"),
    ("temp_ac_side",          "°C",  "temperature",  "Temp AC Side",         "mdi:thermometer",        "diagnostic"),
    ("temp_transformer",      "°C",  "temperature",  "Temp Trafo",           "mdi:thermometer",        "diagnostic"),
    ("charge_state",          None,  None,           "Stare Incarcare",      "mdi:battery-charging",   "diagnostic"),
    ("machine_state",         None,  None,           "Stare Invertor",       "mdi:information",        "diagnostic"),
    ("rtc_datetime",          None,  "timestamp",    "RTC Invertor",         "mdi:clock",              "diagnostic"),
    ("e204_fault",            None,  None,           "Cod Fault",            "mdi:alert",              "diagnostic"),
]

DEVICE = {
    "identifiers":  ["srne_hf2450s80h"],
    "name":         "SRNE Invertor HF2450S80H",
    "model":        "HF2450S80H (Easun ISI Max II 3.6kW/24V)",
    "manufacturer": "SRNE Solar",
    "sw_version":   "Modbus RTU v1.1.0",
}


def publish_discovery(client, cfg: dict):
    prefix = cfg["ha_discovery_prefix"]
    state  = f"{cfg['mqtt_topic_prefix']}/state"

    for key, unit, dc, name, icon, ent_cat in SENSORS:
        p = {
            "name":           name,
            "unique_id":      f"srne_{key}",
            "state_topic":    state,
            "value_template": f"{{{{ value_json.{key} }}}}",
            "device":         DEVICE,
            "icon":           icon,
        }
        if unit:    p["unit_of_measurement"] = unit
        if dc:      p["device_class"] = dc
        if ent_cat: p["entity_category"] = ent_cat
        if unit == "kWh":
            p["state_class"] = "total_increasing"
        elif unit in ("W", "VA", "V", "A", "%", "Hz"):
            p["state_class"] = "measurement"

        client.publish(f"{prefix}/sensor/srne_{key}/config",
                       json.dumps(p), retain=True)

    # Binary sensor fault
    client.publish(f"{prefix}/binary_sensor/srne_fault_active/config", json.dumps({
        "name":           "Fault Activ Invertor",
        "unique_id":      "srne_fault_active",
        "state_topic":    state,
        "value_template": "{{ 'ON' if value_json.fault_active else 'OFF' }}",
        "device_class":   "problem",
        "device":         DEVICE,
        "entity_category":"diagnostic",
    }), retain=True)

    logging.info("HA auto-discovery publicat.")


def publish_state(client, topic_prefix: str, data: dict):
    client.publish(f"{topic_prefix}/state",
                   json.dumps(data, default=str), retain=False)

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    cfg = load_config()
    setup_logging(cfg.get("log_level", "INFO"))

    logging.info("=" * 55)
    logging.info("  SRNE Invertor Modbus v1.1.0")
    logging.info(f"  Log: {LOG_FILE}")
    logging.info(f"  Port: {cfg['serial_port']} | Addr: {cfg['modbus_address']} | Poll: {cfg['poll_interval']}s")
    logging.info("=" * 55)

    # MQTT
    mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                     client_id="srne_invertor_addon")
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

    # Serial
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

    poll      = int(cfg["poll_interval"])
    errors    = 0
    last_rtc  = time.time()

    logging.info(f"Polling activ (interval {poll}s)...")

    while True:
        t0 = time.time()
        try:
            data = read_all(mb)

            if data is None:
                errors += 1
                logging.warning(f"Citire esuata ({errors}/5)")
                if errors >= 5:
                    logging.error("5 erori consecutive — reconectare serial...")
                    mb.disconnect()
                    time.sleep(5)
                    mb.connect()
                    errors = 0
            else:
                errors = 0
                if connected:
                    publish_state(mq, cfg["mqtt_topic_prefix"], data)
                    logging.info(
                        f"SOC={data.get('battery_soc')}% "
                        f"Vbat={data.get('battery_voltage')}V "
                        f"Ibat={data.get('battery_current')}A "
                        f"Ppv={data.get('pv_power')}W "
                        f"Pac={data.get('ac_active_power')}W "
                        f"Tdc={data.get('temp_dc_side')}C "
                        f"State={data.get('machine_state')}"
                    )
                else:
                    logging.warning("MQTT neconectat — date necomunicate")

        except Exception as e:
            logging.exception(f"Eroare neasteptata: {e}")
            errors += 1

        # Sync RTC periodic (la fiecare ora)
        if time.time() - last_rtc >= 3600:
            mb.sync_rtc()
            last_rtc = time.time()

        time.sleep(max(0, poll - (time.time() - t0)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Oprire la cerere.")
        sys.exit(0)
