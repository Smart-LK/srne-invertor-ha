# Changelog — SRNE Invertor Modbus HA Addon

## [3.0.0] — 2026-05-03

### Bazat pe scan complet P00-P10 (2026-05-03, protocol SRNE v1.96)

#### Added
- **MachineState v1.96** la 0x0210: `5=Inverter operation` (mai precis decat blocul vechi)
- **Fault Records (P10)**: toate 32 records citite la startup si 1/zi; 4 active gasite pe hw
- **Write commands via MQTT** (`srne/cmd/`): output_priority, chg_source, power_on/off, clear_faults, clear_stats, equalize_charge
- **HA Buttons si Number entities** pentru write commands
- **Load energy 7-day history** (F01C-F022, confirmat)
- **Senzori noi**: fault_count, latest_fault_desc, latest_fault_time
- **Senzori noi**: model_code, cpu_build_time, hw_ctrl_version
- **Senzori noi**: chg_source_priority, grid_voltage, grid_frequency
- **Senzori noi**: bat_under_volt_v, bat_improve_chg_volt_v
- **slow_poll_interval** configurabil in config.yaml (default 3600s)
- RTC sync inteligent: verifica drift, sincronizeaza doar daca > 60s, zilnic la 00:05

#### Fixed
- Temperaturi accesibile DOAR via blocul 0x0204 x 31 (0x0220+ standalone → exception_0x02)
- Eliminat E01F-E04D (timed chg/dischg) → exception_0x02 pe HF2450S80H
- Eliminat E21C-E221 → exception_0x02
- Eliminat E400-E437 (grid connection) → exception_0x02 (off-grid model)
- Filtrate valori 0x7FFF (registri neinitializati: E215, E216, E218)
- MachineState enum corectat per v1.96 (5=Inverter operation vs 9=Running)
- ChargeState corect la 0x010B (nu 0x010C)

#### Changed
- `srne_modbus.py`: structura refacuta cu `read_fast()` si `read_slow()`
- `config.yaml`: adaugat `slow_poll_interval` in options/schema
- Fault records citite separat (nu in slow poll), refresh 24h
- Battery total Ah (total_increasing) prezent in HA Energy Dashboard

---

## [2.0.0] — 2026-05-02

### Bazat pe scan initial (2026-05-02)

#### Added
- Battery charge/discharge Ah azi (F02D, F02E)
- Battery charge/discharge total Ah 32-bit (F034-F037) = 17572 Ah confirmat DessMonitor
- PV/Load total kWh 32-bit (F038-F03B)
- Work times: inv_work_today_min (F03E), inv_work_total_h (F04A)
- Product info: serial_number (0x0035), fw_app_version, fw_boot_version
- 7-day PV/BatChg/BatDchg history (F000-F01B)
- Battery settings: bat_type (LiFePO4 x9), voltage thresholds
- Output priority si chg source (E200/E20F)

#### Fixed
- E-registers in blocuri <= 10 regs (>22 → timeout pe HF2450S80H)
- ChargeState la 0x010B (per v1.96)
- E004 = BatType (per v1.96), nu MachineState

---

## [1.x.x] — 2026-05-01

### Versiuni intermediare

- v1.3.1: fix output_priority, model info
- v1.3.0: RTC smart sync zilnic la 00:05
- v1.2.0: eliminat temp_controller/temp_battery (0 pe HF2450S80H)
- v1.1.0: recv_slave_frame cu filtrare adresa bus, Ibat signed, log fisier
- v1.0.0: versiune initiala
