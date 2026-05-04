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
  [Addon: SRNE Invertor Modbus v3.0.1]
  Master Modbus RTU, 9600 8N1, poll 30s
        |
  [core-mosquitto MQTT broker]
  Topic prefix: srne/
        |
  [Home Assistant entities + Energy Dashboard]
```

---

## Register Map confirmat pe HF2450S80H (scan 2026-05-03)

### P01 — Date DC (fast poll, 0x0100 x 15)

| Adresa | Descriere | Scala | Nota |
|--------|-----------|-------|------|
| 0x0100 | SOC baterie | % | |
| 0x0101 | Tensiune baterie | x0.1 V | |
| 0x0102 | Curent baterie | signed x0.1 A | neg=incarcare |
| 0x0107 | Tensiune PV1 | x0.1 V | |
| 0x0108 | Curent PV1 | x0.1 A | |
| 0x0109 | Putere PV1 | W | |
| 0x010B | **ChargeState** | enum | 0=Off,1=Quick,2=ConstV,4=Float,6=Li,8=Full |
| 0x010E | Total chg power | W | |
| 0x010F+ | PV2 | N/A | **exception_0x02 pe HF2450S80H** |

### P02 — Date AC (fast poll)

**0x0210 x 16 — Format v1.96 (confirmat):**

| Adresa | Descriere | Scala |
|--------|-----------|-------|
| 0x0210 | **MachineState v1.96** | 5=Inverter operation |
| 0x0213 | Tensiune retea | x0.1 V |
| 0x0215 | Frecventa retea | x0.01 Hz |
| 0x0216 | Tensiune AC out | x0.1 V |
| 0x0218 | Frecventa AC out | x0.01 Hz |
| 0x0219 | Curent AC out | x0.1 A |
| 0x021B | Putere activa AC | W |
| 0x021C | Putere aparenta AC | VA |
| 0x021E | Curent incarcare retea | x0.1 A |
| 0x021F | Sarcina % | % |

**0x0204 x 31 — Format vechi firmware (singura sursa pentru temperaturi!):**

| Adresa | Descriere |
|--------|-----------|
| 0x020C-020E | RTC (citire+scriere) |
| 0x0220 | Temp DC side (x0.1 °C) |
| 0x0221 | Temp AC side (x0.1 °C) |
| 0x0222 | Temp trafo (x0.1 °C) |

> **Important:** 0x0220+ cititi standalone dau `exception_0x02`. Temperaturile sunt accesibile DOAR in blocul 0x0204 x 31.

### P09 — Statistici

| Adresa | Descriere | Scala | state_class |
|--------|-----------|-------|-------------|
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
| F01C-F022 | **Load energy 7 days** | x0.1 kWh/zi | |
| F04A | Inv work total | h | total_increasing |

### P10 — Fault Records (F800-F9F0)

32 inregistrari x 16 regs. `[0]=fault_code, [1-3]=time, [4-15]=data snapshot`.
Citite la startup si 1/zi. 4 faulturi active pe hardware.

---

## Instalare

Settings → Add-ons → Add-on Store → ⋮ → Repositories:
```
https://github.com/Smart-LK/srne-invertor-ha
```

### Configurare

| Camp | Default | Descriere |
|------|---------|-----------|
| `serial_port` | `/dev/ttyUSB1` | Port serial |
| `modbus_address` | `1` | Adresa Modbus |
| `poll_interval` | `30` | Fast poll (s) |
| `slow_poll_interval` | `3600` | Slow poll (s) |
| `mqtt_host` | `core-mosquitto` | Broker MQTT |
| `log_level` | `INFO` | Nivel log |

---

## Entitati HA publicate

### Senzori principali

| Entitate | Unitate | state_class | Nota |
|----------|---------|-------------|------|
| SOC Baterie | % | measurement | |
| Tensiune/Curent Baterie | V, A | measurement | |
| Incarcare/Descarcare Bat Azi | Ah | measurement | |
| **Incarcare Bat Total** | Ah | total_increasing | Valoare exacta |
| **Descarcare Bat Total** | Ah | total_increasing | Valoare exacta |
| **Incarcare Bat Total kWh** | kWh | total_increasing | **HA Energy Dashboard** |
| **Descarcare Bat Total kWh** | kWh | total_increasing | **HA Energy Dashboard** |
| Energie PV Azi | kWh | measurement | |
| **Energie PV Total** | kWh | total_increasing | **HA Energy Dashboard** |
| Consum Sarcina Azi | kWh | measurement | |
| **Consum Sarcina Total** | kWh | total_increasing | **HA Energy Dashboard** |
| Temp DC/AC/Trafo | °C | measurement | |
| Stare Invertor | text | — | MachineState v1.96 |
| Vref Bat (kWh calc) | V | diagnostic | Tensiunea folosita pt kWh |
| Numar Faulturi | — | — | |
| Ultimul Fault / Timp | text | — | |

---

## HA Energy Dashboard

Pentru configurarea diagramei energetice:

- **Solar panels:** `Energie PV Total` (pv_energy_total_kwh)
- **Home consumption:** `Consum Sarcina Total` (load_energy_total_kwh)
- **Battery charged:** `Incarcare Bat Total kWh` (battery_charge_total_kwh)
- **Battery discharged:** `Descarcare Bat Total kWh` (battery_discharge_total_kwh)

### Nota calcul kWh baterie

Invertorul SRNE stocheaza energia bateriei in **Ah** (F034-F037), nu in kWh.
Conversia se face cu tensiunea de referinta **Vref = Ah × Vref / 1000**.

Vref se calculeaza automat in urmatoarea ordine de prioritate:

1. **Media float + over-discharge** (din registri reali E009, E00D):
   - Reprezinta tensiunea medie reala de operare a pack-ului
   - Exemplu 9S LiFePO4: (31.6V + 27.0V) / 2 = **29.3V**
2. **Din tipul bateriei** (N celule × V_nominala/celula):
   - LiFePO4: 3.2V/celula, Ternary: 3.6V/celula
   - Exemplu LiFePO4 x9: 9 × 3.2V = **28.8V**
3. **E003 nominal** (selectia sistemului 12/24/48V) — ultimul resort

Senzorul diagnostic `Vref Bat (kWh calc)` arata exact ce valoare este folosita.

---

## Comenzi MQTT

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
```

---

## Diagnosticare

```bash
# Scan complet toti registrii
python3 srne_scan_v2.py /dev/ttyUSB1

# Scan sectiune specifica
python3 srne_scan_v2.py /dev/ttyUSB1 --area p09   # statistici
python3 srne_scan_v2.py /dev/ttyUSB1 --area p10   # fault records
python3 srne_scan_v2.py /dev/ttyUSB1 --area p05   # setari baterie
```

---

## Changelog

### v3.0.1 — 2026-05-04
- **FIX:** Adaugat `battery_charge_total_kwh` si `battery_discharge_total_kwh`
  (unit=kWh, device_class=energy, state_class=total_increasing)
- **FIX:** Senzori kWh apar acum corect in dropdown **HA Energy Dashboard** > Sistem de baterii
- **NEW:** `compute_bat_v_ref()`: calcul Vref cu prioritate corecta:
  1. Media(bat_float_chg_volt_v E009, bat_over_dischg_volt_v E00D) = 29.3V pt 9S LiFePO4
  2. BAT_TYPE_VNOM[bat_type_code]: N celule × Vcell (LiFePO4 x9 = 28.8V)
  3. bat_nominal_volt_v E003 (12/24/48V) - ultimul resort
- **NEW:** `BAT_TYPE_VNOM` lookup dict pentru toate tipurile de baterie
- **NEW:** Senzor diagnostic `battery_v_ref_v` — arata Vref folosita
- Senzorii in Ah pastrati (battery_charge_total_ah etc.) pentru valoarea exacta

### v3.0.0 — 2026-05-03
- Scan complet P00-P10, implementare finala
- MachineState v1.96 (0x0210), Fault records (P10), Write commands MQTT
- Load energy 7-day history (F01C-F022)

### v2.0.0 — 2026-05-02
- Rewrite complet bazat pe scan confirmat

### v1.0.0 — 2026-05-01
- Versiune initiala
