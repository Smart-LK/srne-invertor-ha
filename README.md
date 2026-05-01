# SRNE/Easun Invertor — Home Assistant Addon

Addon Home Assistant pentru monitorizarea invertorului SRNE HF2450S80H (Easun ISI Max II 3.6kW/24V) via Modbus RTU, cu publicare MQTT și auto-discovery complet în HA.

---

## Hardware testat

| Componentă | Detalii |
|------------|---------|
| Invertor | Easun ISI Max II 3.6kW/24V = SRNE HF2450S80H |
| String PV | 10 panouri în serie (~382V Voc) |
| Baterie | LiFePO4 24V (JBD BMS 9S3P) |
| Interfață USB | Port USB-B pe invertor (mufa pătrată cu colțuri tăiate) |
| Chip serial | CH340 (idVendor=1a86, idProduct=7523) — fără serial number unic |

---

## Schema de conectare Invertor → HA

```
┌─────────────────────────────────────────────────────────────┐
│              Invertor SRNE HF2450S80H                       │
│                                                             │
│   Port USB-B (mufa patrata cu colturi taiate)               │
│   ┌──────────────────────────────┐                          │
│   │  CH340 USB-Serial intern     │  ← chip fara serial ID  │
│   │  prezinta interfata          │                          │
│   │  Modbus RTU SLAVE (addr=1)  │                          │
│   └──────────────────────────────┘                          │
└──────────────────────────────────────────────────────────────┘
         │
         │ Cablu USB-B → USB-A (sau USB-B → USB-C cu adaptor)
         │
         ▼
┌──────────────────────────────────────────────────────────────┐
│   Home Assistant OS (Dell Wyse 5070)                        │
│   /dev/ttyUSB1  (CH340, fara by-id stabil)                  │
│                                                             │
│   ┌─────────────────────────────────────────────────┐       │
│   │  Addon: SRNE Invertor Modbus                    │       │
│   │  Master Modbus RTU, 9600 bps 8N1                │       │
│   │  Poll interval: 30s                             │       │
│   └─────────────────────┬───────────────────────────┘       │
│                         │ MQTT                              │
│   ┌─────────────────────▼───────────────────────────┐       │
│   │  Broker: core-mosquitto                          │       │
│   │  Topic prefix: srne/                            │       │
│   └─────────────────────┬───────────────────────────┘       │
│                         │                                   │
│   ┌─────────────────────▼───────────────────────────┐       │
│   │  Home Assistant                                  │       │
│   │  Auto-discovery → Entitati + Dashboard           │       │
│   └─────────────────────────────────────────────────┘       │
└──────────────────────────────────────────────────────────────┘
```

### Nota despre portul serial

CH340-ul din invertor **nu are serial number unic** — nu apare în `/dev/serial/by-id/` cu un nume identificabil. La fiecare reboot poate lua numărul `ttyUSB0` sau `ttyUSB1` în funcție de ordinea de enumerare USB.

**Soluție practică:** Folosește un port USB fix pe placa HA (de ex. cel mai din stânga), sau configurează udev rules dacă ai mai multe dispozitive CH340.

---

## Protocol Modbus RTU

| Parametru | Valoare |
|-----------|---------|
| Tip | Modbus RTU (serial) |
| Adresă slave | 1 |
| Baud rate | 9600 bps |
| Format | 8N1 (8 data bits, No parity, 1 stop bit) |
| Function codes suportate | FC03 (Read), FC06 (Write single), FC16 (Write multiple) |
| CRC | CRC-16/IBM (Modbus standard), LSB first |

---

## Register Map — confirmat pe firmware HF2450S80H

> **Important:** Registrii sunt dependenți de firmware. Valorile de mai jos sunt confirmate empiric pe acest model. Unii registri din documentul oficial SRNE v3.9 nu există pe firmware-ul invertorului (ex. 0x0113, 0x0121 returnează exception 0x02).

### Bloc 0x0100 — Baterie + String PV
**Citire:** `01 03 01 00 00 0F [CRC]` (15 registri)

| Adresă | Descriere | Scală | Unitate | Note |
|--------|-----------|-------|---------|------|
| 0x0100 | SOC baterie | byte low | % | 0-100 |
| 0x0101 | Tensiune baterie | ÷10 | V | ex. 0x0137=311=31.1V |
| 0x0102 | Curent baterie | **signed** ÷10 | A | negativ=încărcare, pozitiv=descărcare |
| 0x0103 | Temperaturi | byte high=ctrl, low=bat | °C | bit7=semn |
| 0x0104 | Tensiune DC load | ÷10 | V | |
| 0x0105 | Curent DC load | ÷100 | A | |
| 0x0106 | Putere DC load | direct | W | |
| 0x0107 | Tensiune string PV | ÷10 | V | ex. 3828=382.8V (10 panouri serie) |
| 0x0108 | Curent PV | ÷100 | A | |
| 0x0109 | Putere PV | direct | W | |
| 0x010A | Load on/off | 0/1 | — | starea sarcinii |
| 0x010B | Vbat min azi | ÷10 | V | |
| 0x010C | Charging step | enum | — | 0=Off,1=Active,2=MPPT,4=Boost,5=Float |

### Bloc 0x0204 — Ieșire AC + Temperaturi + RTC
**Citire:** `01 03 02 04 00 1F [CRC]` (31 registri)

| Adresă | Descriere | Scală | Unitate | Note |
|--------|-----------|-------|---------|------|
| 0x0209 | Machine state | enum | — | 0=Standby,1=NoAnomaly,9=Running |
| 0x020C | RTC: year/month | byte high=year-2002, low=month | — | |
| 0x020D | RTC: day/hour | byte high=day, low=hour | — | |
| 0x020E | RTC: min/sec | byte high=min, low=sec | — | |
| 0x0210 | Load ratio | direct | % | 0-100 |
| 0x0212 | Running timer | direct | s | crește cu 1/s de la pornire |
| 0x0216 | Tensiune AC output | ÷10 | V | ex. 2300=230.0V |
| 0x0218 | Frecvență AC output | ÷100 | Hz | ex. 4999=49.99Hz |
| 0x0219 | Curent AC output | ÷10 | A | |
| 0x021B | Putere activă AC | direct | W | |
| 0x021C | Putere aparentă AC | direct | VA | |
| 0x0220 | Temperatură DC side | ÷10 | °C | trafo DC |
| 0x0221 | Temperatură AC side | ÷10 | °C | trafo AC |
| 0x0222 | Temperatură trafo | ÷10 | °C | |

**Machine state:**

| Cod | Stare |
|-----|-------|
| 0 | Standby |
| 1 | No anomaly |
| 2 | SW startup |
| 3 | Starting |
| 4 | Running (line/mains mode) |
| 5 | Running (inverter mode) |
| 6 | Running (ECO mode) |
| 7 | Fault |
| 8 | Shutdown |
| 9 | Running (inverter) |

### Bloc 0xF02F — Energie
**Citire:** `01 03 F0 2F 00 0D [CRC]` (13 registri)

| Adresă | Descriere | Scală | Unitate |
|--------|-----------|-------|---------|
| 0xF02F | Energie PV azi | ÷10 | kWh |
| 0xF030 | Consum sarcina azi | ÷10 | kWh |
| 0xF031 | neidentificat | — | — |
| 0xF032 | neidentificat | — | — |
| 0xF038 | Energie PV total cumulativ | ÷10 | kWh |
| 0xF03A | Consum sarcina total cumulativ | ÷10 | kWh |

### E-Registri — Stare și parametri
**Citire individuală FC03, scriere FC06/FC16**

| Adresă | Descriere | Valori confirmate |
|--------|-----------|-------------------|
| 0xE004 | Machine state (read-only) | 9=Running inverter |
| 0xE204 | Fault/alarm | 0=OK |
| 0xE208 | Tensiune AC setata | ÷10=V, ex. 2300=230V |
| 0xE209 | Frecventa AC setata | ÷100=Hz, ex. 5000=50Hz |

### Registri care NU există pe HF2450S80H
Returnează **exception 0x02** (adresă invalidă):

| Adresă | Descriere (doc SRNE) | Status |
|--------|---------------------|--------|
| 0x0113 | Energie PV azi (Wh) | N/A — folosește 0xF02F |
| 0x0114 | Consum azi (Wh) | N/A — folosește 0xF030 |
| 0x0121 | Fault word (32 bit high) | N/A — folosește 0xE204 |
| 0x0122 | Fault word (32 bit low) | N/A — folosește 0xE204 |

---

## Instalare în Home Assistant

### 1. Adaugă repository-ul
Settings → Add-ons → Add-on Store → ⋮ → Repositories → adaugă:
```
https://github.com/Smart-LK/srne-invertor-ha
```

### 2. Instalează și configurează
Refresh → **SRNE Invertor Modbus** → Install → Configuration tab:

| Câmp | Descriere | Default |
|------|-----------|---------|
| `serial_port` | Port serial al invertorului | `/dev/ttyUSB1` |
| `modbus_address` | Adresa Modbus slave | `1` |
| `poll_interval` | Interval citire (secunde) | `30` |
| `mqtt_host` | Broker MQTT | `core-mosquitto` |
| `mqtt_topic_prefix` | Prefix topic MQTT | `srne` |
| `log_level` | Nivel logare | `INFO` |

### 3. Identificare port serial
Din **SSH terminal HA**:
```bash
ls /dev/ttyUSB*
# CH340 fara serial ID — nu apare in by-id cu nume unic
# Verifici care e invertorul si care e alt dispozitiv prin deconectare/reconectare
```

---

## Entități publicate în Home Assistant

### Senzori
| Entitate | Unitate | Descriere |
|----------|---------|-----------|
| SOC Baterie | % | State of Charge |
| Tensiune Baterie | V | Tensiunea pack-ului |
| Curent Baterie | A | Negativ=încărcare, pozitiv=descărcare |
| Temp Controller | °C | Temperatura circuitului de control |
| Temp Baterie | °C | Temperatura bateriei |
| Tensiune PV | V | Tensiunea stringului PV (ex. 382V=10 panouri) |
| Curent PV | A | Curentul de la panouri |
| Putere PV | W | Puterea instantanee PV |
| Energie PV Azi | kWh | Producție PV zilnică |
| Energie PV Total | kWh | Producție PV cumulativă |
| Tensiune AC Out | V | Tensiunea de ieșire AC |
| Frecvență AC Out | Hz | Frecvența de ieșire AC |
| Curent AC Out | A | Curentul de ieșire AC |
| Putere Activă AC | W | Puterea activă livrată |
| Putere Aparentă AC | VA | Puterea aparentă |
| Factor Putere | — | cos φ calculat |
| Sarcina % | % | Procentaj sarcina față de nominal |
| Consum Sarcina Azi | kWh | Consum zilnic |
| Consum Sarcina Total | kWh | Consum cumulativ |
| Temp DC Side | °C | Temperatura latura DC |
| Temp AC Side | °C | Temperatura latura AC |
| Temp Trafo | °C | Temperatura transformator |
| Stare Încărcare | text | Off/MPPT/Boost/Float etc. |
| Stare Invertor | text | Standby/Running etc. |
| RTC Invertor | datetime | Ceasul intern al invertorului |
| Cod Fault | int | 0=OK |

### Binary sensor
| Entitate | Descriere |
|----------|-----------|
| Fault Activ Invertor | ON dacă există orice fault activ |

---

## Diagnosticare — srne_debug.py

Script de diagnosticare rulabil direct din terminalul SSH pe HA, fără a opri addon-ul.

```bash
# Copiere script pe HA
cd /config/addons/srne_invertor

# Test complet (toate blocurile)
python3 srne_debug.py /dev/ttyUSB1

# Citire registru specific
python3 srne_debug.py /dev/ttyUSB1 --reg 0x0100 15
python3 srne_debug.py /dev/ttyUSB1 --reg 0xE004 1

# Scriere parametru
python3 srne_debug.py /dev/ttyUSB1 --write 0xE208 2300

# Scanare porturi disponibile
python3 srne_debug.py --scan

# Dump hex brut
python3 srne_debug.py /dev/ttyUSB1 --raw
```

Log salvat automat în același folder: `srne_debug.log`  
Accesibil via Samba: `\\192.168.200.250\addons\srne_invertor\srne_debug.log`

---

## Changelog

### addon v1.0.0 / srne_debug.py v1.4 — 2026-05-01
- Versiune inițială funcțională confirmată pe HF2450S80H
- Ibat interpretat corect ca signed int16 (negativ=încărcare)
- Registri confirmați: 0x0100×15, 0x0204×31, 0xF02F×13, 0xE004, 0xE204
- Eliminat 0x0113 și 0x0121 din polling (nu există pe acest firmware)
- Vpv confirmat ca tensiune string PV (382V = 10 panouri serie)
- srne_debug.py: log automat în folder script, filtrare bus după addr=0x01 + CRC
- srne_debug.py: coloana Dec(s) în --reg pentru vizualizare signed values

### srne_debug.py v1.3 — 2026-05-01
- Log relativ la folder script (nu /tmp)

### srne_debug.py v1.2 — 2026-05-01
- Log dual consola + fișier
- Filtrare receive după adresa slave + CRC

### srne_debug.py v1.1 — 2026-05-01
- Receive inteligent, citire 0x0100 cu 15 regs (evită exception)

### srne_debug.py v1.0 — 2026-05-01
- Versiune inițială
