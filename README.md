# SRNE/Easun Invertor — Home Assistant Addon

Addon Home Assistant pentru monitorizarea si controlul invertorului SRNE via Modbus RTU, MQTT si HA auto-discovery.

**Testat pe:** Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H, firmware APP V6.64 (Jun 2022)  
**Protocol de referinta:** SRNE MODBUS Energy Storage Inverter v1.96 (Jan 2024)

---

## Hardware testat

| Componenta | Detalii |
|------------|---------|
| Invertor | Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H |
| Serial | SR-2211150019-300917 |
| Firmware | APP V6.64, Boot V2.01, HW V2.00 |
| String PV | 10 panouri in serie (~382V Voc) |
| Baterie | LiFePO4 24V 9S3P (JBD BMS) |
| Interfata USB | Port USB-B (mufa patrata) pe invertor |
| Chip serial | CH340 (idVendor=1a86) — fara serial number unic |

---

## Schema de conectare

```
[Invertor SRNE HF2450S80H]
  Port USB-B -> CH340 intern -> Modbus RTU slave (addr=1)
        |
        | Cablu USB-B -> USB-A
        |
[Home Assistant OS]
  /dev/ttyUSB1  (CH340, fara by-id stabil — poate varia la reboot)
        |
  [Addon: SRNE Invertor Modbus v3.0.0]
  Master Modbus RTU, 9600 8N1, poll 30s
        |
  [core-mosquitto MQTT broker]
  Topic prefix: srne/
        |
  [Home Assistant entities + Energy Dashboard]
```

**Nota port serial:** CH340-ul din invertor nu are serial number unic si nu apare in `/dev/serial/by-id/` cu un nume stabil. Foloseste un port USB fix pe placa HA sau configureaza udev rules.

---

## Protocol Modbus RTU

| Parametru | Valoare |
|-----------|---------|
| Tip | Modbus RTU (serial) |
| Adresa slave | 1 |
| Baud rate | 9600 bps |
| Format | 8N1 |
| FC suportate | FC03 (Read), FC06 (Write single), FC16 (Write multiple) |
| Max registri/cerere | 32 |
| CRC | CRC-16/IBM, LSB first |

---

## Register Map confirmat pe HF2450S80H (scan 2026-05-03)

### P01 — Date DC (fast poll, 0x0100 x 15)

| Adresa | Descriere | Scala | Nota |
|--------|-----------|-------|------|
| 0x0100 | SOC baterie | % | 0-100 |
| 0x0101 | Tensiune baterie | x0.1 V | |
| 0x0102 | Curent baterie | signed x0.1 A | neg=incarcare, poz=descarcare |
| 0x0107 | Tensiune PV1 | x0.1 V | |
| 0x0108 | Curent PV1 | x0.1 A | |
| 0x0109 | Putere PV1 | W | |
| 0x010B | **ChargeState** | enum | 0=Off,1=Quick,2=ConstV,4=Float,6=Li,8=Full |
| 0x010E | Total chg power | W | |
| 0x010F+ | PV2 | N/A | exception_0x02 pe HF2450S80H |

### P02 — Date AC (fast poll)

**0x0210 x 16 — Format v1.96 (confirmat, MachineState precis):**

| Adresa | Descriere | Scala |
|--------|-----------|-------|
| 0x0210 | **MachineState v1.96** | enum: 5=Inverter operation |
| 0x0213 | Tensiune retea (grid) | x0.1 V |
| 0x0215 | Frecventa retea | x0.01 Hz |
| 0x0216 | Tensiune AC out | x0.1 V |
| 0x0218 | Frecventa AC out | x0.01 Hz |
| 0x0219 | Curent AC out | x0.1 A |
| 0x021B | Putere activa AC | W |
| 0x021C | Putere aparenta AC | VA |
| 0x021E | Curent incarcare retea | x0.1 A |
| 0x021F | Sarcina % | % |

**0x0204 x 31 — Format vechi firmware (singura sursa pentru temperaturi!):**

| Adresa | Descriere | Nota |
|--------|-----------|------|
| 0x020C-020E | RTC | citire+scriere |
| 0x0220 | Temp DC side | x0.1 °C |
| 0x0221 | Temp AC side | x0.1 °C |
| 0x0222 | Temp trafo | x0.1 °C |

> **Important:** Registrii 0x0220+ cititi standalone dau `exception_0x02`. Temperaturile sunt accesibile DOAR ca parte a blocului 0x0204 x 31.

### P09 — Statistici (fast + slow poll)

| Adresa | Descriere | Scala | Tip |
|--------|-----------|-------|-----|
| F02D | Bat chg azi | Ah | measurement |
| F02E | Bat dischg azi | Ah | measurement |
| F02F | PV azi | x0.1 kWh | measurement |
| F030 | Consum azi | x0.1 kWh | measurement |
| F034-F035 | **Bat chg total** | Ah 32-bit LE | total_increasing |
| F036-F037 | Bat dischg total | Ah 32-bit LE | total_increasing |
| F038-F039 | **PV total** | x0.1 kWh 32-bit | total_increasing |
| F03A-F03B | **Consum total** | x0.1 kWh 32-bit | total_increasing |
| F000-F006 | PV last 7 days | x0.1 kWh/zi | F000=ieri |
| F007-F00D | Bat chg last 7 days | Ah/zi | |
| F00E-F014 | Bat dischg last 7 days | Ah/zi | |
| F01C-F022 | **Load energy 7 days** | x0.1 kWh/zi | F01C=ieri |
| F04A | Inv work total | h | total_increasing |

### P10 — Fault Records (F800-F9F0)

32 inregistrari x 16 registri. Fiecare record: `[0]=fault_code, [1-3]=time(RTC), [4-15]=data snapshot`.

Faulturi gasite pe hw: 4 active (Bat undertemp, Bat overvoltage x2, Load short)

### P03 — Device Control (DF00-DF0D, confirmat accesibil)

| Adresa | Comanda |
|--------|---------|
| DF00 | Power on(1)/off(0) |
| DF02 | Clear stats(0xBB), Clear fault history(0xCC) |
| DF0D | Immediate equalize charge(1) |

### P05 Battery Settings (E000-E01E, E01F+ -> exception)
### P07 Inverter Settings (E200-E21B, E21C+ -> exception)
### P08 Grid Connection (E400+) -> exception (off-grid model)

---

## Instalare in Home Assistant

### 1. Adauga repository-ul
Settings → Add-ons → Add-on Store → ⋮ → Repositories:
```
https://github.com/Smart-LK/srne-invertor-ha
```

### 2. Configurare

| Camp | Descriere | Default |
|------|-----------|---------|
| `serial_port` | Port serial invertor | `/dev/ttyUSB1` |
| `modbus_address` | Adresa Modbus slave | `1` |
| `poll_interval` | Interval citire fast (s) | `30` |
| `slow_poll_interval` | Interval citire slow (s) | `3600` |
| `mqtt_host` | Broker MQTT | `core-mosquitto` |
| `mqtt_topic_prefix` | Prefix topic MQTT | `srne` |
| `log_level` | Nivel log | `INFO` |

### 3. Identificare port serial
```bash
ls /dev/ttyUSB*
# CH340 fara serial ID — verifica prin deconectare/reconectare USB
```

---

## Entitati HA publicate

### Senzori principali
| Entitate | Unitate | state_class |
|----------|---------| -----------|
| SOC Baterie | % | measurement |
| Tensiune/Curent Baterie | V, A | measurement |
| Incarcare/Descarcare Bat Azi | Ah | measurement |
| **Incarcare/Descarcare Bat Total** | Ah | **total_increasing** |
| Tensiune/Curent/Putere PV | V, A, W | measurement |
| **Energie PV Total** | kWh | **total_increasing** |
| Energie PV Azi | kWh | measurement |
| Tensiune/Frecventa/Curent AC Out | V, Hz, A | measurement |
| Putere Activa/Aparenta AC | W, VA | measurement |
| **Consum Sarcina Total** | kWh | **total_increasing** |
| Consum Sarcina Azi | kWh | measurement |
| Temp DC/AC/Trafo | °C | measurement |
| Stare Invertor | text | — |
| Etapa Incarcare | text | — |
| Prioritate Iesire | text | — |
| Sursa Incarcare | text | — |
| Inv Work Total | h | total_increasing |
| Numar Faulturi | — | — |
| Ultimul Fault / Timp | text | — |
| PV/Consum Ieri | kWh | measurement |
| Firmware APP/Boot | — | — |
| Serial Number | — | — |

### Binary sensor
| Entitate | Descriere |
|----------|-----------|
| Fault Activ Invertor | ON daca exista fault activ |

### Butoane (write commands via MQTT)
| Entitate | Comanda |
|----------|---------|
| Prioritate Iesire (set) | number 0-2: solar/line/sbu |
| Sursa Incarcare (set) | number 0-3: pv/ac/hybrid/pvonly |
| Sterge Faulturi | button -> DF02=0xCC |
| Sterge Statistici | button -> DF02=0xBB |
| Oprire/Pornire Invertor | button -> DF00=0/1 |
| Incarcare Egalizare | button -> DF0D=1 |

---

## HA Energy Dashboard

Pentru diagrama energetica, adauga:
- **Solar panels:** `pv_energy_total_kwh` (total_increasing)
- **Home consumption:** `load_energy_total_kwh` (total_increasing)
- **Battery charge:** `battery_charge_total_ah` (total_increasing, Ah)
- **Battery discharge:** `battery_discharge_total_ah` (total_increasing, Ah)

---

## Comenzi MQTT directe

```bash
# Schimba prioritate iesire: 0=solar, 1=line, 2=sbu
mosquitto_pub -h core-mosquitto -u mqtt_local -P mqtt2026vidra \
  -t srne/cmd/output_priority -m 0

# Sursa incarcare: 0=PV prio, 1=AC prio, 2=hybrid, 3=PV only
mosquitto_pub -h core-mosquitto -u mqtt_local -P mqtt2026vidra \
  -t srne/cmd/chg_source -m 3

# Sterge fault history
mosquitto_pub -h core-mosquitto -u mqtt_local -P mqtt2026vidra \
  -t srne/cmd/clear_faults -m 1

# Pornire/oprire
mosquitto_pub -h core-mosquitto -u mqtt_local -P mqtt2026vidra \
  -t srne/cmd/power_on -m 1
```

---

## Diagnosticare

```bash
# Scan complet toti registrii (toate sectiunile P00-P10)
python3 srne_scan_v2.py /dev/ttyUSB1

# Scan sectiune specifica
python3 srne_scan_v2.py /dev/ttyUSB1 --area p09  # statistici
python3 srne_scan_v2.py /dev/ttyUSB1 --area p10  # fault records
python3 srne_scan_v2.py /dev/ttyUSB1 --area p05  # setari baterie

# Debug interactiv
python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15
```

Log-uri accesibile via Samba: `\\<HA_IP>\addons\srne_invertor\`

---

## Changelog

### v3.0.0 — 2026-05-03
- Scan complet P00-P10, implementare finala bazata 100% pe date confirmate
- **NEW:** 0x0210 MachineState v1.96 (5=Inverter operation, precis)
- **NEW:** F01C-F022 load energy 7-day history (confirmat)
- **NEW:** P10 Fault records citite la startup si 1/zi (32 records, 4 active)
- **NEW:** Write commands via MQTT (output priority, chg source, power on/off, clear faults/stats, equalize)
- **NEW:** Senzori: model_code, cpu_build_time, hw_ctrl_version, fault_count, latest_fault
- **NEW:** Istoricul 7 zile pentru load energy (F01C-F022)
- **FIX:** Temperaturi accesibile DOAR via 0x0204 x 31 (0x0220+ standalone -> exception)
- **FIX:** E01F+, E21C+, E400-E437 -> exception pe HF2450S80H, eliminate
- **FIX:** Valori 0x7FFF filtrate (registri neinitializati pe firmware)
- **FIX:** MachineState enum corectat per v1.96
- **FIX:** slow_poll_interval configurabil in config.yaml

### v2.0.0 — 2026-05-02
- Rewrite complet bazat pe scan confirmat
- Registri baterie Ah azi/total (32-bit), work times
- Product info: serial, firmware, HW versions

### v1.x.x — 2026-05-01
- Versiuni intermediare cu fix-uri iterative

### v1.0.0 — 2026-05-01
- Versiune initiala
