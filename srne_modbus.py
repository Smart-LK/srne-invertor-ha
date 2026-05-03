#!/usr/bin/env python3
"""
srne_modbus.py v3.0.0 - SRNE Invertor Modbus RTU -> MQTT -> Home Assistant
===========================================================================
Dispozitiv testat: Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H
  Serial: SR-2211150019-300917  |  Firmware: APP V6.64 (Jun 2022)
  Boot V2.01  |  HW V2.00  |  ProductType: 4 = All-in-one solar charger

Interfata: Port USB-B (mufa patrata) -> CH340 -> /dev/ttyUSB*
           CH340 fara serial number unic - port poate varia la reboot
Protocol:  Modbus RTU, addr=1, FC03 read, FC06/FC16 write, 9600 8N1
           Max 32 registri per cerere (limita protocol)

===============================================================================
REGISTER MAP CONFIRMAT (scan complet 2026-05-03, protocol v1.96)
===============================================================================

FAST POLL (poll_interval, default 30s):
  P01 0x0100 x 15: SOC, Vbat, Ibat(signed x0.1), Vpv1, Ipv1, Ppv1,
                   PvTotalPwr, ChargeState@0x010B, TotalChgPwr@0x010E
                   Nota: 0x010F+ (Pv2) -> exception_0x02 pe HF2450S80H

  P02 0x0210 x 16: MachineState(v1.96)@0x0210 = 5=Inverter operation
                   GridVoltA@0x0213, GridFreq@0x0215, InvVoltA@0x0216,
                   InvFreq@0x0218, LoadCurrA@0x0219, LoadActivePwr@0x021B,
                   LoadApparentPwr@0x021C, LineChgCurr@0x021E, LoadRatio@0x021F
                   Nota: 0x0220+ standalone -> exception! Temps doar via 0x0204

  P02 0x0204 x 31: Include RTC@020C-020E + Temp DC/AC/Trafo @0x0220-0x0222
                   SINGURA modalitate de citire temperaturi pe acest firmware!

  P02 0x0204 x 4:  CurrFaultCode (0=OK)

  P09 0xF02C x 8:  BatChgToday@F02D(Ah), BatDchgToday@F02E(Ah),
                   PvToday@F02F(kWh x0.1), LoadToday@F030(kWh x0.1)
  P09 0xF034 x 8:  BatChgTotal@F034-F035(Ah,32-bit LE) <- 17641 Ah confirmat
                   BatDchgTotal@F036-F037, PvTotal@F038-F039, LoadTotal@F03A-F03B
  P09 0xF03C x 6:  InvWorkToday@F03E(min)

SLOW POLL (slow_poll_interval, default 3600s):
  P09 0xF000 x 28: PV/BatChg/BatDchg/GridChg ultim 7 zile (F000-F01B)
  P09 0xF01C x 11: Load energy 7 zile (F01C-F022) - CONFIRMAT
  P09 0xF04A x 2:  InvWorkTotal(h), GridWorkTotal(h)
  P05 0xE000 x 5:  BatModel (type, cap, volt, pv chg max)
  P05 0xE005 x 10: Bat voltage thresholds (OverVolt...DischgLimit)
  P05 0xE00F x 16: Bat timers/SOC (E00F-E01E), E01F+ -> exception!
  P07 0xE200 x 10: OutputPriority@E204, VoltSet, FreqSet
  P07 0xE20A x 10: MaxChgCurr, ChgSourcePriority@E20F
  P07 0xE214 x 8:  BMS settings (pana la E21B), E21C+ -> exception!
  P10 0xF800+:     Fault records (32 x 16 regs), citit startup + 1/zi

DEVICE CONTROL (write, confirmat accesibil):
  0xDF00: Power on(1)/off(0)
  0xDF02: Clear stats(0xBB), Clear fault history(0xCC)
  0xDF0D: Immediate equalize charge (1)
  0x020C-020E: RTC sync
  0xE204: Set output priority (0=Solar, 1=Line, 2=SBU)
  0xE20F: Set chg source priority (0=PV prio, 1=AC prio, 2=Hybrid, 3=PV only)

NOT AVAILABLE pe HF2450S80H:
  P01 0x010F-0x0111: Pv2 -> exception_0x02
  P02 0x0220+: Temps standalone -> exception (foloseste 0x0204 block)
  P05 0xE01F-0xE04D: Timed charge/discharge -> exception_0x02
  P07 0xE21C-0xE221: Max line current etc -> exception_0x02
  P08 0xE400-0xE437: Grid connection -> exception (off-grid model)

VALORI DEFAULT 'NESETATE' (filtrate):
  E215=32767, E216=7, E218=32767 -> registri neinitializati pe HW

===============================================================================
HA Energy Dashboard:
  pv_energy_total_kwh, load_energy_total_kwh (total_increasing)
  battery_charge_total_ah, battery_discharge_total_ah (total_increasing)

MQTT Write Commands (publica la srne/cmd/TOPIC):
  srne/cmd/output_priority -> 0/1/2 (solar/line/sbu)
  srne/cmd/chg_source      -> 0/1/2/3
  srne/cmd/power_on, power_off, clear_faults, clear_stats, equalize -> 1

Changelog:
  v3.0.0 - Scan complet 2026-05-03, implementare finala
           NEW: 0x0210x16 MachineState v1.96 (5=Inverter, confirmat)
           NEW: F01C-F022 load energy 7-day history (confirmat)
           NEW: P10 Fault records (32 records, 4 active pe hw)
           NEW: Write commands via MQTT
           NEW: model_code, cpu_build_time, hw_ctrl_version in senzori
           FIX: Temps doar via 0x0204x31 (0x0220+ standalone -> exception)
           FIX: E01F+, E21C+, E400+ remove (exception)
           FIX: Filtrare valori 0x7FFF (nesetate pe HW)
           FIX: MachineState enum per v1.96
  v2.0.0 - Rewrite complet dupa scan initial 2026-05-02
  v1.0.0 - Initial

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
    "slow_poll_interval":  3600,
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

# --- Logging ------------------------------------------------------------------

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "srne_modbus.log")

def setup_logging(level_str: str):
    level = getattr(logging, level_str.upper(), logging.INFO)
    fmt   = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                              datefmt="%Y-%m-%d %H:%M:%S")
    root  = logging.getLogger()
    root.setLevel(level)
    for h in root.handlers[:]:
        root.removeHandler(h)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    root.addHandler(ch)
    try:
        fh = logging.FileHandler(LOG_FILE, mode="a", encoding="utf-8")
        fh.setFormatter(fmt)
        root.addHandler(fh)
    except Exception as e:
        logging.warning(f"Nu pot deschide {LOG_FILE}: {e}")

# --- CRC16 --------------------------------------------------------------------

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def _fc03(addr, reg, cnt):
    pdu = struct.pack(">BBHH", addr, 3, reg, cnt)
    return pdu + struct.pack("<H", _crc16(pdu))

def _fc06(addr, reg, val):
    pdu = struct.pack(">BBHH", addr, 6, reg, val & 0xFFFF)
    return pdu + struct.pack("<H", _crc16(pdu))

def _fc16(addr, reg, values):
    n = len(values)
    pdu = struct.pack(">BBHHB", addr, 0x10, reg, n, n*2)
    for v in values:
        pdu += struct.pack(">H", v & 0xFFFF)
    return pdu + struct.pack("<H", _crc16(pdu))

def _s16(v): return v if v < 0x8000 else v - 0x10000
def _u32(lo, hi): return (hi << 16) | lo
def _dec_str(regs): return ''.join(chr(r&0xFF) if 32<=r&0xFF<127 else '' for r in regs).strip()
def _is_unset(v): return v == 0x7FFF

# --- Receive with slave address filter ----------------------------------------

def _recv_slave_frame(ser, slave_addr: int, expected_regs: int, timeout=3.0):
    buf = bytearray()
    start = time.time()
    ignored = 0
    while time.time() - start < timeout:
        c = ser.read(256)
        if c: buf.extend(c)
        i = 0
        while i < len(buf):
            if buf[i] != slave_addr:
                ignored += 1; i += 1; continue
            rest = buf[i:]
            if len(rest) >= 3 and rest[1] == 3:
                bc = rest[2]
                if bc != expected_regs * 2: i += 1; continue
                tot = 3 + bc + 2
                if len(rest) < tot: break
                frame = bytes(rest[:tot])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0: logging.debug(f"Bus: ignorat {ignored}b")
                    return frame, False
                i += 1; continue
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == _crc16(frame[:-2]):
                    if ignored > 0: logging.debug(f"Bus: ignorat {ignored}b")
                    return frame, True
                i += 1; continue
            i += 1
    if ignored > 0: logging.debug(f"Bus: ignorat {ignored}b (timeout)")
    return None, False

# --- Modbus RTU ---------------------------------------------------------------

class ModbusRTU:
    def __init__(self, port, baudrate=9600, device_addr=1):
        self.port = port; self.device_addr = device_addr
        self._baudrate = baudrate; self._ser = None

    def connect(self):
        self._ser = serial.Serial(self.port, self._baudrate,
            bytesize=8, parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)
        logging.info(f"Serial OK: {self.port} @ {self._baudrate} bps")

    def disconnect(self):
        if self._ser and self._ser.is_open: self._ser.close()

    def read_registers(self, reg: int, cnt: int) -> list | None:
        if cnt > 32: cnt = 32
        request = _fc03(self.device_addr, reg, cnt)
        self._ser.reset_input_buffer(); time.sleep(0.06)
        self._ser.write(request)
        frame, is_exc = _recv_slave_frame(self._ser, self.device_addr, cnt)
        if frame is None:
            logging.warning(f"Timeout FC03 0x{reg:04X}x{cnt}"); return None
        if is_exc:
            logging.warning(f"Exception FC03 0x{reg:04X}: 0x{frame[2]:02X}"); return None
        return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(cnt)]

    def write_register(self, reg: int, val: int) -> bool:
        self._ser.reset_input_buffer()
        self._ser.write(_fc06(self.device_addr, reg, val))
        resp = self._ser.read(8)
        ok = len(resp)==8 and struct.unpack("<H", resp[-2:])[0]==_crc16(resp[:-2])
        logging.info(f"Write {'OK' if ok else 'FAIL'}: 0x{reg:04X}={val}"); return ok

    def write_registers(self, reg: int, values: list) -> bool:
        self._ser.reset_input_buffer()
        self._ser.write(_fc16(self.device_addr, reg, values))
        resp = self._ser.read(8)
        ok = len(resp)==8 and struct.unpack("<H", resp[-2:])[0]==_crc16(resp[:-2])
        logging.info(f"WriteMulti {'OK' if ok else 'FAIL'}: 0x{reg:04X} x{len(values)}"); return ok

    def read_rtc(self) -> datetime | None:
        r = self.read_registers(0x020C, 3)
        if not r: return None
        try:
            r0,r1,r2=r
            return datetime((r0>>8)+2002, r0&0xFF, r1>>8, r1&0xFF, r2>>8, r2&0xFF)
        except: return None

    def sync_rtc(self) -> bool:
        inv = self.read_rtc(); now = datetime.now()
        if inv is None:
            logging.warning("RTC check: nu pot citi")
            return self._write_rtc(now)
        drift = abs((now-inv).total_seconds())
        logging.info(f"RTC check: inv={inv.strftime('%H:%M:%S')} sys={now.strftime('%H:%M:%S')} drift={drift:.0f}s")
        if drift > 60:
            logging.info(f"RTC sync -> {now.strftime('%Y-%m-%d %H:%M:%S')}")
            return self._write_rtc(now)
        logging.info("RTC ok, fara sync"); return False

    def _write_rtc(self, dt: datetime) -> bool:
        yy = dt.year - 2002
        return self.write_registers(0x020C, [
            (yy<<8)|dt.month, (dt.day<<8)|dt.hour, (dt.minute<<8)|dt.second])

# --- Enums --------------------------------------------------------------------

CHARGE_STATE = {0:"Off",1:"Quick charge",2:"Const voltage",4:"Float",6:"Li activate",8:"Full"}

MACHINE_STATE_V196 = {  # confirmed at 0x0210 per v1.96
    0:"Init",1:"Standby",2:"AC power",3:"Inverter",4:"AC power",
    5:"Inverter operation",6:"Inv->AC",7:"AC->Inv",8:"Bat activate",
    9:"Manual shutdown",10:"Fault"
}

BATTERY_TYPE = {
    0:"User define",1:"SLD",2:"FLD",3:"GEL",
    4:"LiFePO4 x14",5:"LiFePO4 x15",6:"LiFePO4 x16",
    7:"LiFePO4 x7",8:"LiFePO4 x8",9:"LiFePO4 x9",
    10:"Ternary x7",11:"Ternary x8",12:"Ternary x13",13:"Ternary x14"
}

OUTPUT_PRIORITY = {0:"Solar",1:"Line",2:"SBU"}
CHG_SOURCE = {0:"PV priority",1:"AC priority",2:"Hybrid",3:"PV only"}

FAULT_CODES = {
    1:"Bat overvoltage",2:"Bat undervoltage",3:"Bat discharge overcurrent",
    4:"Load short",5:"Bat overtemperature",6:"Bat undertemperature",
    7:"Inv overvoltage",8:"Inv undervoltage",9:"Inv overcurrent",
    10:"Bus overvoltage",11:"Bus undervoltage",12:"Inv overload",
    13:"Fan fault",14:"PV overvoltage",15:"PV overcurrent",16:"Bat reverse",
    17:"Bat temp sensor",18:"Inv output short",19:"Grid overvoltage",
    20:"Grid undervoltage",21:"Grid over-freq",22:"Grid under-freq",
    23:"Output inconsistency",24:"Output imbalance"
}

# --- Parse functions ----------------------------------------------------------

def parse_p01_dc(regs: list) -> dict:
    """P01 0x0100 x 15. ChargeState at 0x010B. Max 15 regs!"""
    def r(a): return regs[a-0x0100] if 0<=a-0x0100<len(regs) else 0
    ibat=_s16(r(0x0102)); cs=r(0x010B)&0xFF
    return {
        "battery_soc":         r(0x0100)&0xFF,
        "battery_voltage":     round(r(0x0101)*0.1,1),
        "battery_current":     round(ibat*0.1,1),
        "pv_voltage":          round(r(0x0107)*0.1,1),
        "pv_current":          round(r(0x0108)*0.1,1),
        "pv_power":            r(0x0109),
        "pv_total_power":      r(0x010A),
        "total_charge_power":  r(0x010E),
        "battery_charge_step": CHARGE_STATE.get(cs,f"?({cs})"),
        "battery_charge_step_code": cs,
    }

def parse_p02_ac_v196(regs: list) -> dict:
    """P02 0x0210 x 16. MachineState v1.96 format. Confirmed."""
    def r(a): return regs[a-0x0210] if 0<=a-0x0210<len(regs) else 0
    ms=r(0x0210)&0xFF; pac=r(0x021B); pap=r(0x021C)
    return {
        "machine_state":       MACHINE_STATE_V196.get(ms,f"?({ms})"),
        "machine_state_code":  ms,
        "grid_voltage":        round(r(0x0213)*0.1,1),
        "grid_frequency":      round(r(0x0215)*0.01,2),
        "ac_output_voltage":   round(r(0x0216)*0.1,1),
        "ac_output_current":   round(r(0x0219)*0.1,1),
        "ac_output_frequency": round(r(0x0218)*0.01,2),
        "ac_active_power":     pac,
        "ac_apparent_power":   pap,
        "power_factor":        round(pac/pap,3) if pap else 0.0,
        "load_ratio":          r(0x021F),
        "line_chg_current":    round(r(0x021E)*0.1,1),
    }

def parse_p02_old_temps(regs: list) -> dict:
    """P02 0x0204 x 31. ONLY way to read temps on this firmware."""
    def r(a): return regs[a-0x0204] if 0<=a-0x0204<len(regs) else 0
    r0,r1,r2=r(0x020C),r(0x020D),r(0x020E)
    try: rtc=f"{(r0>>8)+2002:04d}-{r0&0xFF:02d}-{r1>>8:02d}T{r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}"
    except: rtc="invalid"
    return {
        "rtc_datetime":    rtc,
        "temp_dc_side":    round(_s16(r(0x0220))*0.1,1),
        "temp_ac_side":    round(_s16(r(0x0221))*0.1,1),
        "temp_transformer":round(_s16(r(0x0222))*0.1,1),
    }

def parse_f02c(regs: list) -> dict:
    def r(i): return regs[i] if i<len(regs) else 0
    return {
        "pv_to_grid_today_kwh":       round(r(0)*0.1,1),
        "battery_charge_today_ah":    r(1),
        "battery_discharge_today_ah": r(2),
        "pv_energy_today_kwh":        round(r(3)*0.1,1),
        "load_energy_today_kwh":      round(r(4)*0.1,1),
        "operating_days_total":       r(5),
    }

def parse_f034(regs: list) -> dict:
    def r(i): return regs[i] if i<len(regs) else 0
    return {
        "battery_charge_total_ah":    _u32(r(0),r(1)),
        "battery_discharge_total_ah": _u32(r(2),r(3)),
        "pv_energy_total_kwh":        round(_u32(r(4),r(5))*0.1,1),
        "load_energy_total_kwh":      round(_u32(r(6),r(7))*0.1,1),
    }

def parse_f000_history(regs: list) -> dict:
    def r(i): return regs[i] if i<len(regs) else 0
    result={}; labels=["yesterday","2d_ago","3d_ago","4d_ago","5d_ago","6d_ago","7d_ago"]
    for i,lbl in enumerate(labels):
        result[f"pv_energy_{lbl}_kwh"]=round(r(i)*0.1,1)
        result[f"bat_chg_{lbl}_ah"]=r(7+i)
        result[f"bat_dischg_{lbl}_ah"]=r(14+i)
    return result

def parse_f01c_history(regs: list) -> dict:
    def r(i): return regs[i] if i<len(regs) else 0
    result={}; labels=["yesterday","2d_ago","3d_ago","4d_ago","5d_ago","6d_ago","7d_ago"]
    for i,lbl in enumerate(labels):
        result[f"load_energy_{lbl}_kwh"]=round(r(i)*0.1,1)
    return result

def parse_f03c(regs: list) -> dict:
    def r(i): return regs[i] if i<len(regs) else 0
    return {
        "grid_charge_today_ah": r(0),
        "grid_load_today_kwh":  round(r(1)*0.1,1),
        "inv_work_today_min":   r(2),
        "grid_work_today_min":  r(3),
    }

def parse_battery_settings(r1, r2, r3) -> dict:
    result={}
    if r1 and len(r1)>=5:
        bat_v=r1[3]; bt=r1[4]
        result.update({"bat_pv_chg_max_a":r1[1],"bat_nominal_cap_ah":r1[2],
            "bat_nominal_volt_v":bat_v,"bat_type_code":bt,
            "bat_type":BATTERY_TYPE.get(bt,f"?({bt})"),})
    else: bat_v=24
    vf=bat_v/12.0 if (r1 and r1[3]>0) else 2.0
    if r2 and len(r2)>=10:
        vn=["bat_over_volt_v","bat_chg_limit_volt_v","bat_const_chg_volt_v",
            "bat_improve_chg_volt_v","bat_float_chg_volt_v","bat_improve_chg_back_volt_v",
            "bat_over_dischg_back_volt_v","bat_under_volt_v","bat_over_dischg_volt_v",
            "bat_dischg_limit_volt_v"]
        for i,name in enumerate(vn): result[name]=round(r2[i]*0.1*vf,1)
    if r3 and len(r3)>=4:
        result.update({"bat_dischg_stop_soc":r3[0],"bat_overdischg_delay_s":r3[1],
            "bat_const_chg_time_min":r3[2],"bat_improve_chg_time_min":r3[3]})
    return result

def parse_inverter_settings(ri1, ri2, ri3) -> dict:
    result={}
    if ri1 and len(ri1)>=10:
        op=ri1[4]
        result.update({"output_priority_code":op,"output_priority":OUTPUT_PRIORITY.get(op,f"?({op})"),
            "output_volt_set_v":round(ri1[8]*0.1,1),"output_freq_set_hz":round(ri1[9]*0.01,2)})
    if ri2 and len(ri2)>=10:
        max_chg=ri2[0]; cs=ri2[5]
        result.update({"max_chg_current_a":round(max_chg*0.1,1) if not _is_unset(max_chg) else None,
            "chg_source_priority_code":cs,"chg_source_priority":CHG_SOURCE.get(cs,f"?({cs})")})
    if ri3 and len(ri3)>=8:
        bms=ri3[7]
        if not _is_unset(bms): result["bms_protocol"]=bms
    return {k:v for k,v in result.items() if v is not None}

def read_fault_records(mb: ModbusRTU) -> list:
    faults=[]
    for rec in range(32):
        base=0xF800+rec*0x10
        r=mb.read_registers(base,16)
        if r is None: continue
        fc=r[0]
        if fc==0: continue
        r0,r1,r2=r[1],r[2],r[3]
        try: yr=(r0>>8)+2002; t=f"{yr}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}"
        except: t="?"
        faults.append({"record":rec,"code":fc,"desc":FAULT_CODES.get(fc,f"code {fc}"),
            "time":t,"data":[f"0x{v:04X}" for v in r[4:16]]})
        time.sleep(0.05)
    logging.info(f"Fault records: {len(faults)} active")
    return faults

def read_product_info(mb: ModbusRTU) -> dict:
    info={}
    r=mb.read_registers(0x000A,2)
    if r: info["product_type_code"]=r[1]
    time.sleep(0.1)
    r=mb.read_registers(0x0014,3)
    if r: info["fw_app_version"]=round(r[0]/100,2); info["fw_boot_version"]=round(r[1]/100,2); info["hw_ctrl_version"]=round(r[2]/100,2)
    time.sleep(0.1)
    r=mb.read_registers(0x001A,2)
    if r: info["model_code"]=r[1]
    time.sleep(0.1)
    r=mb.read_registers(0x0021,20)
    if r: info["cpu_build_time"]=_dec_str(r)
    time.sleep(0.1)
    r=mb.read_registers(0x0035,20)
    if r: info["serial_number"]=_dec_str(r)
    time.sleep(0.1)
    if info: logging.info(f"Product: SN={info.get('serial_number')} APP=V{info.get('fw_app_version')} Boot=V{info.get('fw_boot_version')}")
    return info

# --- Fast poll ----------------------------------------------------------------

def read_fast(mb: ModbusRTU) -> dict | None:
    result={"timestamp":datetime.now().isoformat()}
    r=mb.read_registers(0x0100,15)
    if r is None: return None
    result.update(parse_p01_dc(r)); time.sleep(0.12)
    r=mb.read_registers(0x0210,16)
    if r: result.update(parse_p02_ac_v196(r)); time.sleep(0.12)
    r=mb.read_registers(0x0204,31)
    if r: result.update(parse_p02_old_temps(r)); time.sleep(0.12)
    r=mb.read_registers(0x0204,4)
    if r:
        fc=[r[i] for i in range(4)]
        result["fault_codes_raw"]=fc; result["fault_active"]=any(c!=0 for c in fc)
    else: result["fault_active"]=False
    time.sleep(0.1)
    r=mb.read_registers(0xF02C,8)
    if r: result.update(parse_f02c(r)); time.sleep(0.1)
    r=mb.read_registers(0xF034,8)
    if r: result.update(parse_f034(r)); time.sleep(0.1)
    r=mb.read_registers(0xF03C,6)
    if r: result.update(parse_f03c(r))
    return result

# --- Slow poll ----------------------------------------------------------------

def read_slow(mb: ModbusRTU, cache: dict) -> dict:
    result={}
    r=mb.read_registers(0xF000,28)
    if r:
        h=parse_f000_history(r); result.update(h); cache.update(h)
    else: result.update({k:v for k,v in cache.items() if "yesterday" in k or "d_ago" in k})
    time.sleep(0.12)
    r=mb.read_registers(0xF01C,11)
    if r:
        h=parse_f01c_history(r); result.update(h); cache.update(h)
    else: result.update({k:v for k,v in cache.items() if k.startswith("load_energy_") and "d_ago" in k})
    time.sleep(0.12)
    r=mb.read_registers(0xF04A,2)
    if r:
        result["inv_work_total_h"]=r[0]; result["grid_work_total_h"]=r[1]
        cache["inv_work_total_h"]=r[0]; cache["grid_work_total_h"]=r[1]
    elif "inv_work_total_h" in cache:
        result["inv_work_total_h"]=cache["inv_work_total_h"]
        result["grid_work_total_h"]=cache.get("grid_work_total_h",0)
    time.sleep(0.12)
    r1=mb.read_registers(0xE000,5); time.sleep(0.1)
    r2=mb.read_registers(0xE005,10); time.sleep(0.1)
    r3=mb.read_registers(0xE00F,16)
    bs=parse_battery_settings(r1,r2,r3); result.update(bs); cache.update(bs); time.sleep(0.12)
    ri1=mb.read_registers(0xE200,10); time.sleep(0.1)
    ri2=mb.read_registers(0xE20A,10); time.sleep(0.1)
    ri3=mb.read_registers(0xE214,8)
    is_=parse_inverter_settings(ri1,ri2,ri3); result.update(is_); cache.update(is_); time.sleep(0.12)
    return result

# --- MQTT + HA Discovery ------------------------------------------------------

# (key, unit, dc, name, icon, ent_cat, precision, sc_override)
# sc_override: None=auto, "total_increasing", "measurement", ""=no state_class
SENSORS = [
    # Battery
    ("battery_soc","%","battery","SOC Baterie","mdi:battery",None,0,None),
    ("battery_voltage","V","voltage","Tensiune Baterie","mdi:battery-charging",None,1,None),
    ("battery_current","A","current","Curent Baterie","mdi:current-dc",None,1,None),
    ("battery_charge_today_ah","Ah",None,"Incarcare Bat Azi","mdi:battery-arrow-up",None,0,"measurement"),
    ("battery_discharge_today_ah","Ah",None,"Descarcare Bat Azi","mdi:battery-arrow-down",None,0,"measurement"),
    ("battery_charge_total_ah","Ah",None,"Incarcare Bat Total","mdi:battery-plus",None,0,"total_increasing"),
    ("battery_discharge_total_ah","Ah",None,"Descarcare Bat Total","mdi:battery-minus",None,0,"total_increasing"),
    # PV
    ("pv_voltage","V","voltage","Tensiune PV","mdi:solar-panel",None,1,None),
    ("pv_current","A","current","Curent PV","mdi:solar-panel",None,1,None),
    ("pv_power","W","power","Putere PV","mdi:solar-power",None,0,None),
    ("pv_energy_today_kwh","kWh","energy","Energie PV Azi","mdi:solar-power",None,1,"measurement"),
    ("pv_energy_total_kwh","kWh","energy","Energie PV Total","mdi:solar-power",None,1,"total_increasing"),
    # AC Output
    ("ac_output_voltage","V","voltage","Tensiune AC Out","mdi:power-plug",None,1,None),
    ("ac_output_frequency","Hz","frequency","Frecventa AC Out","mdi:sine-wave",None,2,None),
    ("ac_output_current","A","current","Curent AC Out","mdi:current-ac",None,1,None),
    ("ac_active_power","W","power","Putere Activa AC","mdi:lightning-bolt",None,0,None),
    ("ac_apparent_power","VA",None,"Putere Aparenta AC","mdi:lightning-bolt",None,0,None),
    ("power_factor",None,"power_factor","Factor Putere","mdi:angle-acute",None,3,None),
    ("load_ratio","%",None,"Sarcina %","mdi:gauge",None,0,None),
    ("line_chg_current","A","current","Curent Incarcare Retea","mdi:transmission-tower",None,1,None),
    ("load_energy_today_kwh","kWh","energy","Consum Sarcina Azi","mdi:home-lightning-bolt",None,1,"measurement"),
    ("load_energy_total_kwh","kWh","energy","Consum Sarcina Total","mdi:home-lightning-bolt",None,1,"total_increasing"),
    # Temperatures
    ("temp_dc_side","\u00b0C","temperature","Temp DC Side","mdi:thermometer","diagnostic",1,None),
    ("temp_ac_side","\u00b0C","temperature","Temp AC Side","mdi:thermometer","diagnostic",1,None),
    ("temp_transformer","\u00b0C","temperature","Temp Trafo","mdi:thermometer","diagnostic",1,None),
    # Status
    ("battery_charge_step",None,None,"Etapa Incarcare","mdi:battery-charging","diagnostic",None,""),
    ("machine_state",None,None,"Stare Invertor","mdi:information","diagnostic",None,""),
    ("output_priority",None,None,"Prioritate Iesire","mdi:priority-high","diagnostic",None,""),
    ("chg_source_priority",None,None,"Sursa Incarcare","mdi:solar-panel-large","diagnostic",None,""),
    ("grid_voltage","V","voltage","Tensiune Retea","mdi:transmission-tower","diagnostic",1,None),
    ("grid_frequency","Hz","frequency","Frecventa Retea","mdi:sine-wave","diagnostic",2,None),
    # Work time
    ("inv_work_today_min","min",None,"Inv Work Azi","mdi:timer","diagnostic",0,"measurement"),
    ("inv_work_total_h","h",None,"Inv Work Total","mdi:timer","diagnostic",0,"total_increasing"),
    # Product / Firmware
    ("fw_app_version",None,None,"Firmware APP","mdi:chip","diagnostic",2,""),
    ("fw_boot_version",None,None,"Firmware Boot","mdi:chip","diagnostic",2,""),
    ("serial_number",None,None,"Serial Number","mdi:barcode","diagnostic",None,""),
    ("model_code",None,None,"Cod Model","mdi:identifier","diagnostic",None,""),
    ("cpu_build_time",None,None,"Firmware Build Date","mdi:calendar-clock","diagnostic",None,""),
    # Battery settings (slow poll)
    ("bat_pv_chg_max_a","A",None,"PV I Max Incarcare","mdi:solar-panel","diagnostic",0,""),
    ("bat_nominal_cap_ah","Ah",None,"Capacitate Nominala","mdi:battery","diagnostic",0,""),
    ("bat_nominal_volt_v","V",None,"Tensiune Nominala Bat","mdi:battery","diagnostic",0,""),
    ("bat_type",None,None,"Tip Baterie","mdi:battery-heart","diagnostic",None,""),
    ("bat_float_chg_volt_v","V","voltage","Tensiune Float","mdi:battery-charging","diagnostic",1,""),
    ("bat_over_dischg_volt_v","V","voltage","Tensiune OverDischg","mdi:battery-alert","diagnostic",1,""),
    ("bat_under_volt_v","V","voltage","Tensiune UnderVolt","mdi:battery-low","diagnostic",1,""),
    # Faults
    ("fault_count",None,None,"Numar Faulturi","mdi:alert-circle","diagnostic",0,""),
    ("latest_fault_desc",None,None,"Ultimul Fault","mdi:alert","diagnostic",None,""),
    ("latest_fault_time",None,None,"Timp Ultim Fault","mdi:clock-alert","diagnostic",None,""),
    # 7-day history
    ("pv_energy_yesterday_kwh","kWh","energy","PV Ieri","mdi:solar-power","diagnostic",1,"measurement"),
    ("load_energy_yesterday_kwh","kWh","energy","Consum Ieri","mdi:home-lightning-bolt","diagnostic",1,"measurement"),
    ("bat_chg_yesterday_ah","Ah",None,"Incarcare Bat Ieri","mdi:battery-arrow-up","diagnostic",0,"measurement"),
    ("bat_dischg_yesterday_ah","Ah",None,"Descarcare Bat Ieri","mdi:battery-arrow-down","diagnostic",0,"measurement"),
]

def _make_device(pi: dict) -> dict:
    sn=pi.get("serial_number",""); app=pi.get("fw_app_version","?")
    return {"identifiers":[f"srne_{sn}" if sn else "srne_hf2450s80h"],
        "name":"SRNE Invertor","model":"HF2450S80H (Easun ISI Max II 3.6kW/24V)",
        "manufacturer":"SRNE Solar","serial_number":sn,"sw_version":f"APP V{app}"}

def publish_discovery(client, cfg: dict, pi: dict):
    prefix=cfg["ha_discovery_prefix"]; state=f"{cfg['mqtt_topic_prefix']}/state"
    device=_make_device(pi); cmd=f"{cfg['mqtt_topic_prefix']}/cmd"

    for key,unit,dc,name,icon,ent_cat,precision,sc_override in SENSORS:
        p={"name":name,"unique_id":f"srne_{key}","state_topic":state,
           "value_template":f"{{{{ value_json.{key} }}}}","device":device,"icon":icon}
        if unit: p["unit_of_measurement"]=unit
        if dc: p["device_class"]=dc
        if ent_cat: p["entity_category"]=ent_cat
        if precision is not None: p["suggested_display_precision"]=precision
        if sc_override is None:
            if unit=="kWh" and "total" in key: p["state_class"]="total_increasing"
            elif unit=="Ah" and "total" in key: p["state_class"]="total_increasing"
            elif "total_h" in key: p["state_class"]="total_increasing"
            elif unit in ("W","VA","V","A","%","Hz","\u00b0C","Ah","min"): p["state_class"]="measurement"
        elif sc_override: p["state_class"]=sc_override
        client.publish(f"{prefix}/sensor/srne_{key}/config",json.dumps(p),retain=True)

    client.publish(f"{prefix}/binary_sensor/srne_fault_active/config",json.dumps({
        "name":"Fault Activ Invertor","unique_id":"srne_fault_active",
        "state_topic":state,"value_template":"{{ 'ON' if value_json.fault_active else 'OFF' }}",
        "device_class":"problem","device":device,"entity_category":"diagnostic"}),retain=True)

    for key,name,vmin,vmax in [("output_priority","Prioritate Iesire (set)",0,2),("chg_source","Sursa Incarcare (set)",0,3)]:
        client.publish(f"{prefix}/number/srne_{key}_set/config",json.dumps({
            "name":name,"unique_id":f"srne_set_{key}","command_topic":f"{cmd}/{key}",
            "min":vmin,"max":vmax,"step":1,"mode":"box","device":device,
            "entity_category":"config","icon":"mdi:cog"}),retain=True)

    for key,name in [("clear_faults","Sterge Faulturi"),("clear_stats","Sterge Statistici"),
                     ("power_off","Oprire Invertor"),("power_on","Pornire Invertor"),("equalize","Egalizare")]:
        client.publish(f"{prefix}/button/srne_{key}/config",json.dumps({
            "name":name,"unique_id":f"srne_btn_{key}","command_topic":f"{cmd}/{key}",
            "payload_press":"1","device":device,"entity_category":"config",
            "icon":"mdi:gesture-tap"}),retain=True)

    logging.info("HA auto-discovery publicat.")

def publish_state(client, topic_prefix: str, data: dict):
    client.publish(f"{topic_prefix}/state",json.dumps(data,default=str),retain=False)

def handle_cmd(mb: ModbusRTU, topic: str, payload: str):
    try: val=int(payload.strip())
    except: logging.warning(f"Cmd invalid: '{payload}'"); return
    cmd=topic.split("/")[-1]
    if cmd=="output_priority" and 0<=val<=2: mb.write_register(0xE204,val)
    elif cmd=="chg_source" and 0<=val<=3: mb.write_register(0xE20F,val)
    elif cmd=="power_on": mb.write_register(0xDF00,1)
    elif cmd=="power_off": mb.write_register(0xDF00,0)
    elif cmd=="power_onoff": mb.write_register(0xDF00,1 if val else 0)
    elif cmd=="clear_faults": mb.write_register(0xDF02,0xCC); logging.info("Fault history cleared")
    elif cmd=="clear_stats": mb.write_register(0xDF02,0xBB); logging.info("Statistics cleared")
    elif cmd=="equalize": mb.write_register(0xDF0D,1); logging.info("Equalize charge started")
    else: logging.warning(f"Cmd necunoscut: {cmd}")

# --- Main ---------------------------------------------------------------------

RTC_CHECK_HOUR=0; RTC_CHECK_MINUTE=5
FAULT_READ_INTERVAL=86400

def main():
    cfg=load_config(); setup_logging(cfg.get("log_level","INFO"))
    slow=cfg.get("slow_poll_interval",DEFAULTS["slow_poll_interval"])
    logging.info("="*58)
    logging.info("  SRNE Invertor Modbus v3.0.0")
    logging.info(f"  Log: {LOG_FILE}")
    logging.info(f"  Port: {cfg['serial_port']} | Poll: {cfg['poll_interval']}s | Slow: {slow}s")
    logging.info("="*58)

    mq=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,client_id="srne_invertor_addon")
    mq.username_pw_set(cfg["mqtt_user"],cfg["mqtt_password"])
    connected=False; mb_ref=[None]

    def on_connect(c,u,f,rc,p=None):
        nonlocal connected; connected=(rc==0)
        logging.info(f"MQTT {'OK' if connected else f'FAIL rc={rc}'}")
        if connected: c.subscribe(f"{cfg['mqtt_topic_prefix']}/cmd/#")
    def on_disconnect(c,u,rc,p=None):
        nonlocal connected; connected=False; logging.warning("MQTT deconectat")
    def on_message(c,u,msg):
        if mb_ref[0]: handle_cmd(mb_ref[0],msg.topic,msg.payload.decode(errors="ignore"))

    mq.on_connect=on_connect; mq.on_disconnect=on_disconnect; mq.on_message=on_message

    while True:
        try: mq.connect(cfg["mqtt_host"],cfg["mqtt_port"],keepalive=60); break
        except Exception as e: logging.error(f"MQTT: {e}. Retry 10s..."); time.sleep(10)
    mq.loop_start(); time.sleep(2)

    mb=ModbusRTU(cfg["serial_port"],device_addr=cfg["modbus_address"])
    mb_ref[0]=mb
    while True:
        try: mb.connect(); break
        except serial.SerialException as e: logging.error(f"Serial: {e}. Retry 15s..."); time.sleep(15)

    logging.info("Citire product info...")
    product_info=read_product_info(mb)
    logging.info("Citire fault records (startup)...")
    fault_records=read_fault_records(mb); last_fault_read=time.time()
    if connected: publish_discovery(mq,cfg,product_info)

    slow_cache={}; slow_cache.update(product_info); last_slow=0.0
    logging.info("Citire slow registers (initial)...")
    slow_data=read_slow(mb,slow_cache); slow_data.update(product_info); last_slow=time.time()
    rtc_synced_date=None; errors=0; poll=int(cfg["poll_interval"]); slow_i=int(slow)
    logging.info(f"Polling: fast={poll}s slow={slow_i}s")

    while True:
        t0=time.time(); now=datetime.now()
        today=now.date()
        if now.hour==RTC_CHECK_HOUR and now.minute==RTC_CHECK_MINUTE and rtc_synced_date!=today:
            rtc_synced_date=today; mb.sync_rtc()
        if t0-last_slow>=slow_i:
            slow_data=read_slow(mb,slow_cache); slow_data.update(product_info); last_slow=t0
        if t0-last_fault_read>=FAULT_READ_INTERVAL:
            fault_records=read_fault_records(mb); last_fault_read=t0
        try:
            data=read_fast(mb)
            if data is None:
                errors+=1; logging.warning(f"Citire critica esuata ({errors}/5)")
                if errors>=5:
                    logging.error("5 erori -> reconectare..."); mb.disconnect(); time.sleep(5); mb.connect(); errors=0
            else:
                errors=0; data.update(slow_data)
                data["fault_count"]=len(fault_records)
                if fault_records:
                    latest=fault_records[-1]
                    data["latest_fault_desc"]=latest["desc"]; data["latest_fault_time"]=latest["time"]
                else:
                    data["latest_fault_desc"]="None"; data["latest_fault_time"]="None"
                data["pv_energy_yesterday_kwh"]=data.get("pv_energy_yesterday_kwh",0)
                data["load_energy_yesterday_kwh"]=data.get("load_energy_yesterday_kwh",0)
                data["bat_chg_yesterday_ah"]=data.get("bat_chg_yesterday_ah",0)
                data["bat_dischg_yesterday_ah"]=data.get("bat_dischg_yesterday_ah",0)
                if connected:
                    publish_state(mq,cfg["mqtt_topic_prefix"],data)
                    logging.info(f"SOC={data.get('battery_soc')}% Vbat={data.get('battery_voltage')}V "
                        f"Ibat={data.get('battery_current')}A Ppv={data.get('pv_power')}W "
                        f"Pac={data.get('ac_active_power')}W Tdc={data.get('temp_dc_side')}C "
                        f"State={data.get('machine_state')} BatAzi={data.get('battery_charge_today_ah')}Ah "
                        f"BatTot={data.get('battery_charge_total_ah')}Ah")
                else: logging.warning("MQTT neconectat")
        except Exception as e: logging.exception(f"Eroare: {e}"); errors+=1
        time.sleep(max(0,poll-(time.time()-t0)))

if __name__=="__main__":
    try: main()
    except KeyboardInterrupt: logging.info("Oprire."); sys.exit(0)
