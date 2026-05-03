#!/usr/bin/env python3
"""
srne_scan_v2.py - Scanare COMPLETA registri SRNE Energy Storage Inverter
=========================================================================
Conform: MODBUS Protocol for Energy Storage Inverter v1.96 (Jan 2024)

Testeaza TOATE sectiunile din protocol si raporteaza ce exista pe hardware.
Log complet: srne_scan_v2.log langa script.

Rulare:
  pip install pyserial
  python3 srne_scan_v2.py /dev/ttyUSB1               # toate sectiunile
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p00    # P00 product info
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p01    # P01 DC data
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p02    # P02 AC inverter data
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p03    # P03 device control
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p05    # P05 battery settings
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p07    # P07 inverter user settings
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p08    # P08 grid connection
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p09    # P09 power statistics
  python3 srne_scan_v2.py /dev/ttyUSB1 --area p10    # P10 fault records (32x)

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import sys, time, struct, argparse, os, logging
from datetime import datetime

try:
    import serial
except ImportError:
    print("Lipsa: pip install pyserial"); sys.exit(1)

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "srne_scan_v2.log")

def setup_logger():
    lg = logging.getLogger("scan")
    lg.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S")
    lg.addHandler(logging.StreamHandler(sys.stdout))
    lg.handlers[0].setFormatter(fmt)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt)
    lg.addHandler(fh)
    return lg

log = logging.getLogger("scan")
def p(m=""): log.info(m)

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def fc03(addr, reg, cnt):
    pdu = struct.pack(">BBHH", addr, 3, reg, cnt)
    return pdu + struct.pack("<H", crc16(pdu))

def recv(ser, addr, cnt, timeout=3.0):
    buf = bytearray(); start = time.time(); ignored = 0
    while time.time() - start < timeout:
        c = ser.read(256)
        if c: buf.extend(c)
        i = 0
        while i < len(buf):
            if buf[i] != addr: ignored += 1; i += 1; continue
            rest = buf[i:]
            if len(rest) >= 3 and rest[1] == 3:
                bc = rest[2]
                if bc != cnt*2: i += 1; continue
                tot = 3+bc+2
                if len(rest) < tot: break
                frame = bytes(rest[:tot])
                if struct.unpack("<H", frame[-2:])[0] == crc16(frame[:-2]):
                    return frame, False
                i += 1; continue
            if len(rest) >= 5 and rest[1] == 0x83:
                frame = bytes(rest[:5])
                if struct.unpack("<H", frame[-2:])[0] == crc16(frame[:-2]):
                    return frame, True
                i += 1; continue
            i += 1
    return None, False

def read_regs(ser, addr, reg, cnt, timeout=3.0):
    if cnt > 32: return None, "count>32"
    ser.reset_input_buffer(); time.sleep(0.06)
    ser.write(fc03(addr, reg, cnt))
    frame, exc = recv(ser, addr, cnt, timeout)
    if frame is None: return None, "timeout"
    if exc: return None, f"exception_0x{frame[2]:02X}"
    return [struct.unpack(">H", frame[3+i*2:5+i*2])[0] for i in range(cnt)], "ok"

def s16(v): return v if v < 0x8000 else v - 0x10000
def u32(lo, hi): return (hi << 16) | lo
def dec_str(regs): return ''.join(chr(r&0xFF) if 32<=r&0xFF<127 else '?' for r in regs if r&0xFF).strip()

BATTERY_TYPE = {0:"User define",1:"SLD",2:"FLD",3:"GEL",
    4:"LiFePO4 x14",5:"LiFePO4 x15",6:"LiFePO4 x16",
    7:"LiFePO4 x7",8:"LiFePO4 x8",9:"LiFePO4 x9",
    10:"Ternary x7",11:"Ternary x8",12:"Ternary x13",13:"Ternary x14"}
CHARGE_STATE = {0:"Off",1:"Quick charge",2:"Const voltage",4:"Float",6:"Li activate",8:"Full"}
MACHINE_STATE = {0:"Init",1:"Standby",2:"AC power",3:"Inverter",4:"AC power",5:"Inverter",
    6:"Inv->AC",7:"AC->Inv",8:"Bat activate",9:"Manual shutdown",10:"Fault"}
OUTPUT_PRIORITY = {0:"Solar",1:"Line",2:"SBU",3:"Solar only/Off-grid"}
CHG_SOURCE = {0:"PV prio (AC backup)",1:"AC prio (PV backup)",2:"Hybrid (PV prio)",3:"PV only"}
PRODUCT_TYPE = {0:"Domestic ctrl",1:"Street light",3:"Grid-connected",4:"All-in-one solar",5:"Off-grid"}
FAULT_CODES = {1:"Bat overvoltage",2:"Bat undervoltage",3:"Bat discharge overcurrent",
    4:"Load short",5:"Bat overtemp",6:"Bat undertemp",7:"Inv overvoltage",8:"Inv undervoltage",
    9:"Inv overcurrent",10:"Bus overvoltage",11:"Bus undervoltage",12:"Inv overload",
    13:"Fan fault",14:"PV overvoltage",15:"PV overcurrent",16:"Bat reverse",
    17:"Bat temp sensor",18:"Inv output short",19:"Grid overvolt",20:"Grid undervolt",
    21:"Grid overfreq",22:"Grid underfreq",23:"Output inconsistency",24:"Output imbalance"}

def scan_p00(ser, addr):
    p(); p("="*65); p("  P00 - PRODUCT INFORMATION (0x000A-0x0048)"); p("="*65)
    r,s=read_regs(ser,addr,0x000A,2)
    if s=="ok": p(f"  [000A] MinorVersion: {r[0]}"); p(f"  [000B] ProductType:  {r[1]} = {PRODUCT_TYPE.get(r[1],f'?({r[1]})')}")
    else: p(f"  [000A-000B] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x0014,4)
    if s=="ok":
        p(f"  [0014] APP version:    {r[0]} = V{r[0]/100:.2f}")
        p(f"  [0015] BOOT version:   {r[1]} = V{r[1]/100:.2f}")
        p(f"  [0016] HW ctrl:        {r[2]} = V{r[2]/100:.2f}")
        p(f"  [0017] HW power:       {r[3]} = V{r[3]/100:.2f}")
    else: p(f"  [0014-0017] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x001A,8)
    if s=="ok":
        p(f"  [001A] RS485 addr:     {r[0]}"); p(f"  [001B] Model code:     {r[1]}")
        p(f"  [001C] Protocol ver:   {r[2]} = V{r[2]/100:.2f}")
        yr=(r[4]>>8)+2000
        p(f"  [001E] Manuf date:     {yr}-{r[4]&0xFF:02d}-{r[5]>>8:02d} {r[5]&0xFF:02d}h")
        p(f"  [0020] Product area:   {['Shenzhen','Dongguan'][r[6]] if r[6]<2 else r[6]}")
    else: p(f"  [001A-0020] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x0021,20)
    p(f"  [0021-0034] CPU build: '{dec_str(r)}'" if s=="ok" else f"  [0021-0034] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x0035,20)
    p(f"  [0035-0048] Serial No: '{dec_str(r)}'" if s=="ok" else f"  [0035-0048] -> {s}")

def scan_p01(ser, addr):
    p(); p("="*65); p("  P01 - DC DATA (0x0100-0x0111)"); p("="*65)
    r,s=read_regs(ser,addr,0x0100,15)
    if s=="ok":
        ibat=s16(r[2]); cs=r[11]&0xFF
        p(f"  [0100] SOC:            {r[0]&0xFF}%")
        p(f"  [0101] Vbat:           {r[1]*0.1:.1f} V  (raw {r[1]:#06x})")
        p(f"  [0102] Ibat signed:    {ibat*0.1:.1f} A  ({'discharge' if ibat>0 else 'charge' if ibat<0 else 'idle'})")
        p(f"  [0103] Tbat:           {s16(r[3])*0.1:.1f} C  (raw {r[3]:#06x})")
        p(f"  [0104-0106] reserved:  {r[4:7]}")
        p(f"  [0107] Vpv1:           {r[7]*0.1:.1f} V")
        p(f"  [0108] Ipv1:           {r[8]*0.1:.1f} A")
        p(f"  [0109] Ppv1:           {r[9]} W")
        p(f"  [010A] PvTotalPower:   {r[10]} W")
        p(f"  [010B] ChargeState:    {cs} = {CHARGE_STATE.get(cs,f'?({cs})')}")
        p(f"  [010C-010D] reserved:  {r[12:14]}")
        p(f"  [010E] TotalChgPower:  {r[14]} W")
    else: p(f"  [0100-010E] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x010F,3)
    if s=="ok": p(f"  [010F] Vpv2: {r[0]*0.1:.1f}V  [0110] Ipv2: {r[1]*0.1:.1f}A  [0111] Ppv2: {r[2]}W")
    else: p(f"  [010F-0111] Pv2 -> {s}")

def scan_p02(ser, addr):
    p(); p("="*65); p("  P02 - INVERTER DATA (0x0200-0x0243)"); p("="*65)
    r,s=read_regs(ser,addr,0x0200,4)
    p(f"  [0200-0203] CurrErrReg:  {[f'0x{v:04X}' for v in r]}" if s=="ok" else f"  [0200-0203] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x0204,4)
    if s=="ok":
        for i,v in enumerate(r):
            p(f"  [020{4+i}] FaultCode{i}: {v} {'= '+FAULT_CODES.get(v,'?') if v else '= OK'}")
    else: p(f"  [0204-0207] FaultCodes -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x020C,3)
    if s=="ok":
        r0,r1,r2=r
        try: p(f"  [020C-020E] RTC: {(r0>>8)+2002}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}  *** CONFIRMED ***")
        except: p(f"  [020C-020E] RTC raw: {r}")
    else: p(f"  [020C-020E] RTC -> {s}")
    time.sleep(0.1)
    # 0x0210 new format AC data block (v1.96)
    r,s=read_regs(ser,addr,0x0210,16)
    if s=="ok":
        ms=r[0]
        p(f"  [0210] MachineState:   {ms} = {MACHINE_STATE.get(ms,f'?({ms})')}  *** v1.96 format ***")
        p(f"  [0212] BusVoltSum:     {r[2]*0.1:.1f} V")
        p(f"  [0213] GridVoltA:      {r[3]*0.1:.1f} V")
        p(f"  [0215] GridFreq:       {r[5]*0.01:.2f} Hz")
        p(f"  [0216] InvVoltA:       {r[6]*0.1:.1f} V")
        p(f"  [0218] InvFreq:        {r[8]*0.01:.2f} Hz")
        p(f"  [0219] LoadCurrA:      {r[9]*0.1:.1f} A")
        p(f"  [021B] LoadActivePwr:  {r[11]} W")
        p(f"  [021C] LoadAppPwr:     {r[12]} VA")
        p(f"  [021E] LineChgCurr:    {r[14]*0.1:.1f} A")
        p(f"  [021F] LoadRatioA:     {r[15]}%")
    else: p(f"  [0210-021F] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x0220,10)
    if s=="ok":
        p(f"  [0220] Temp DC-DC:     {s16(r[0])*0.1:.1f} C")
        p(f"  [0221] Temp DC-AC:     {s16(r[1])*0.1:.1f} C")
        p(f"  [0222] Temp Trafo:     {s16(r[2])*0.1:.1f} C")
        p(f"  [0223] Temp Ambient:   {s16(r[3])*0.1:.1f} C")
        p(f"  [0224] PV->bat chg:    {r[4]*0.1:.1f} A")
        p(f"  [0226] InvFaultState:  0x{r[6]:04X}")
    else: p(f"  [0220-0229] -> {s}")
    time.sleep(0.1)
    # Test remaining: 022A-0243
    r,s=read_regs(ser,addr,0x022A,16)
    if s=="ok":
        p(f"  [022A] GridVoltB:      {r[0]*0.1:.1f} V")
        p(f"  [022B] GridVoltC:      {r[1]*0.1:.1f} V")
        p(f"  [023A] GridActivePwrA: {s16(r[16]) if len(r)>16 else 'N/A'} W")
    else: p(f"  [022A-0239] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0x023A,10)
    if s=="ok":
        p(f"  [023A] GridActivePwrA: {s16(r[0])} W  [023B] B: {s16(r[1])} W  [023C] C: {s16(r[2])} W")
        p(f"  [0240] HomeLoadA:      {r[6]} W  [0241] B: {r[7]} W  [0242] C: {r[8]} W")
    else: p(f"  [023A-0243] -> {s}")

def scan_p03(ser, addr):
    p(); p("="*65); p("  P03 - DEVICE CONTROL (0xDF00-0xDF0D)  [READ to test existence]"); p("="*65)
    r,s=read_regs(ser,addr,0xDF00,14)
    if s=="ok":
        names=["PowerOnOff","MachineReset","RestoreFactory","Rsvd0","Rsvd1","Rsvd2",
               "UpgradeH","UpgradeL","Rsvd3","Rsvd4H","Rsvd4M","Rsvd4L","Rsvd5","BattEqualChgImm"]
        for i,(n,v) in enumerate(zip(names,r)):
            p(f"  [DF{i:02X}] {n:20s}: {v}")
    else: p(f"  [DF00-DF0D] -> {s}")
    p("  Write cmds: DF00=1:ON/0:OFF  DF01=1:Reset  DF02=0xAA:Factory/0xBB:ClearStats/0xCC:ClearFaults  DF0D=1:EqualChg")

def scan_p05(ser, addr):
    p(); p("="*65); p("  P05 - BATTERY SETTINGS (0xE000-0xE04D)"); p("="*65)
    p("  NOTE: max 10 regs per read on HF2450S80H (>22 -> TIMEOUT!)")
    r,s=read_regs(ser,addr,0xE000,5)
    if s=="ok":
        bat_v=r[3]; vf=bat_v/12.0 if bat_v>0 else 2.0
        p(f"  [E000] Reserved:           {r[0]}")
        p(f"  [E001] PvChgCurrMax:       {r[1]} A")
        p(f"  [E002] BatNomCap:          {r[2]} Ah")
        p(f"  [E003] BatNomVolt:         {bat_v} V")
        p(f"  [E004] BatType:            {r[4]} = {BATTERY_TYPE.get(r[4],f'?({r[4]})')}")
    else: p(f"  [E000-E004] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE005,10)
    if s=="ok":
        bat_v_=24; vf=bat_v_/12.0
        names=["OverVolt","ChgLimit","ConstChg","ImprovChg","Float","ImprovChgBack","OverDischgBack","UnderVolt","OverDischg","DischgLimit"]
        for i,(n,v) in enumerate(zip(names,r)):
            p(f"  [E{5+i:03X}] {n:16s}: {v} raw = {v*0.1:.1f}V(12V) = {v*0.1*vf:.1f}V(24V)")
    else: p(f"  [E005-E00E] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE00F,16)
    if s=="ok":
        p(f"  [E00F] DischgStopSOC:      {r[0]}%")
        p(f"  [E010] OverDischgDelay:    {r[1]} s")
        p(f"  [E011] ConstChgTime:       {r[2]} min")
        p(f"  [E012] ImprovChgTime:      {r[3]} min")
        p(f"  [E013] ConstChgGapTime:    {r[4]} days")
        p(f"  [E014] TempCompCoeff:      {r[5]} mV/C/2")
        p(f"  [E015] ChgMaxTemp:         {r[6]} C")
        p(f"  [E016] ChgMinTemp:         {r[7]} C")
        p(f"  [E017] DischgMaxTemp:      {r[8]} C")
        p(f"  [E018] DischgMinTemp:      {r[9]} C")
        p(f"  [E019] HeatStartTemp:      {r[10]} C")
        p(f"  [E01A] HeatStopTemp:       {r[11]} C")
        p(f"  [E01B] BatSwitchDcVolt:    {r[12]*0.1:.1f} V")
        p(f"  [E01C] StopChgCurr:        {r[13]*0.1:.1f} A")
        p(f"  [E01D] StopChgSOC:         {r[14]}%")
        p(f"  [E01E] SocLowAlarm:        {r[15]}%")
    else: p(f"  [E00F-E01E] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE01F,16)
    if s=="ok":
        p(f"  [E01F] SocSwToLine:        {r[0]}% (SBU mode)")
        p(f"  [E020] SocSwToBatt:        {r[1]}% (SBU mode)")
        p(f"  [E022] BattVoltSwToInv:    {r[3]*0.1:.1f} V")
        p(f"  [E023] EqualChgTimeout:    {r[4]} min")
        p(f"  [E024] LiBattActiveCurr:   {r[5]*0.1:.1f} A")
        p(f"  [E025] BMSChgLCMode:       {r[6]}")
        def hm(v): return f"{v>>8:02d}:{v&0xFF:02d}"
        p(f"  [E026-E027] Chg1:          {hm(r[7])}-{hm(r[8])}")
        p(f"  [E028-E029] Chg2:          {hm(r[9])}-{hm(r[10])}")
        p(f"  [E02A-E02B] Chg3:          {hm(r[11])}-{hm(r[12])}")
        p(f"  [E02C] OnTimeChargeEn:     {r[13]}")
        p(f"  [E02D-E02E] Dischg1:       {hm(r[14])}-{hm(r[15])}")
    else: p(f"  [E01F-E02E] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE02F,16)
    if s=="ok":
        def hm(v): return f"{v>>8:02d}:{v&0xFF:02d}"
        p(f"  [E02F-E030] Dischg2:       {hm(r[0])}-{hm(r[1])}")
        p(f"  [E031-E032] Dischg3:       {hm(r[2])}-{hm(r[3])}")
        p(f"  [E033] OnTimeDischgEn:     {r[4]}")
        p(f"  [E037] WorkMode:           {r[8]} (0=off-grid,1=grid,2=ACout anti-rev,3=ACin anti-rev)")
        p(f"  [E038] LeakageCurrDtcEn:   {r[9]}")
        p(f"  [E039] PvPowerPriority:    {r[10]} (0=chg prio, 1=load prio)")
        p(f"  [E03A] BattTempCompEn:     {r[11]}")
        p(f"  [E03B] TimedChg1StopSOC:   {r[12]}%")
        p(f"  [E03C] TimedChg2StopSOC:   {r[13]}%")
        p(f"  [E03D] TimedChg3StopSOC:   {r[14]}%")
        p(f"  [E03E] TimedDchg1StopSOC:  {r[15]}%")
    else: p(f"  [E02F-E03E] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE03F,15)
    if s=="ok":
        p(f"  [E03F] TimedDchg2StopSOC:  {r[0]}%")
        p(f"  [E040] TimedDchg3StopSOC:  {r[1]}%")
        p(f"  [E041-E043] ChgStopVolt:   {[v*0.1 for v in r[2:5]]} V")
        p(f"  [E044-E046] DchgStopVolt:  {[v*0.1 for v in r[5:8]]} V")
        p(f"  [E047-E049] DchgMaxPwr:    {[v*10 for v in r[8:11]]} W")
        p(f"  [E04A-E04C] ChgMaxPwr:     {[v*10 for v in r[11:14]]} W")
        p(f"  [E04D] TimedChgSource:     {r[14]} (bits: AC/gen per period)")
    else: p(f"  [E03F-E04D] -> {s}")

def scan_p07(ser, addr):
    p(); p("="*65); p("  P07 - INVERTER USER SETTINGS (0xE200-0xE221)"); p("="*65)
    r,s=read_regs(ser,addr,0xE200,10)
    if s=="ok":
        pm=r[1]; op=r[4]
        p(f"  [E200] RS485 addr:         {r[0]}")
        p(f"  [E201] ParallMode:         {pm} = {['Single','1ph//','2ph//','2ph 120','2ph 180','3ph A','3ph B','3ph C'][pm] if pm<8 else pm}")
        p(f"  [E202] Password:           {'(set)' if r[2] else '(none)'}")
        p(f"  [E204] OutputPriority:     {op} = {OUTPUT_PRIORITY.get(op,f'?({op})')}  *** OUTPUT PRIORITY ***")
        p(f"  [E205] IbattLineChgLimit:  {r[5]*0.1:.1f} A")
        p(f"  [E206] EqualChgEnable:     {r[6]}")
        p(f"  [E207] N_G_FuncEn:         {r[7]}")
        p(f"  [E208] OutputVoltSet:      {r[8]*0.1:.1f} V")
        p(f"  [E209] OutputFreqSet:      {r[9]*0.01:.2f} Hz")
    else: p(f"  [E200-E209] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE20A,10)
    if s=="ok":
        p(f"  [E20A] MaxChgCurr:         {r[0]*0.1:.1f} A")
        p(f"  [E20B] AcVoltRange:        {r[1]} (0=wide APL, 1=narrow UPS)")
        p(f"  [E20C] PowerSavingMode:    {r[2]}")
        p(f"  [E20D] AutoRestartOvLoad:  {r[3]}")
        p(f"  [E20E] AutoRestartOvTemp:  {r[4]}")
        p(f"  [E20F] ChgSourcePriority:  {r[5]} = {CHG_SOURCE.get(r[5],f'?({r[5]})')}  *** CHG SOURCE ***")
        p(f"  [E210] AlarmEnable:        {r[6]}")
        p(f"  [E211] AlarmOnSrcLoss:     {r[7]}")
        p(f"  [E212] BypOvLoad:          {r[8]}")
        p(f"  [E213] RecordFaultEn:      {r[9]}")
    else: p(f"  [E20A-E213] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE214,8)
    if s=="ok":
        p(f"  [E214] BmsErrStopEn:       {r[0]}")
        p(f"  [E215] BmsCommEnable:      {r[1]} (0=off,1=485,2=CAN)")
        p(f"  [E216] DcLoadSwitch:       {r[2]} (0=off, 1=on)")
        p(f"  [E218] DeratePower:        {r[4]} W")
        p(f"  [E21B] Rs485BmsProtocol:   {r[7]}")
    else: p(f"  [E214-E21B] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE21C,6)
    if s=="ok":
        p(f"  [E21C] MaxLineCurrent:     {r[0]*0.1:.1f} A")
        p(f"  [E21D] MaxLinePower:       {r[1]*10} W")
        p(f"  [E21E] OutputPhaseSet:     {r[2]}")
        p(f"  [E21F] GenWorkMode:        {r[3]}")
        p(f"  [E220] GenChgMaxCurr:      {r[4]*0.1:.1f} A")
        p(f"  [E221] GenRatePower:       {r[5]} W")
    else: p(f"  [E21C-E221] -> {s}")

def scan_p08(ser, addr):
    p(); p("="*65); p("  P08 - GRID CONNECTION SETTINGS (0xE400-0xE437)"); p("="*65)
    r,s=read_regs(ser,addr,0xE400,16)
    if s=="ok":
        p(f"  [E400] GridActivePwrSet:   {r[0]} W")
        p(f"  [E401] GridPfSet:          {s16(r[1])*0.001:.3f}")
        p(f"  [E402] GridQset:           {s16(r[2])}%")
        p(f"  [E403] GridStandard:       {r[3]}")
        p(f"  [E404] GridUVLevel1:       {r[4]*0.1:.1f} V")
        p(f"  [E405] GridUVTime1:        {r[5]*20} ms")
        p(f"  [E408] GridUVLevel2:       {r[8]*0.1:.1f} V")
        p(f"  [E40C] GridOVLevel1:       {r[12]*0.1:.1f} V")
        p(f"  [E40F] GridOVResumTime1:   {r[15]*20} ms")
    else: p(f"  [E400-E40F] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE410,16)
    if s=="ok":
        p(f"  [E410] GridOVLevel2:       {r[0]*0.1:.1f} V")
        p(f"  [E414] GridUFLevel1:       {r[4]*0.01:.2f} Hz")
        p(f"  [E41C] GridOFLevel1:       {r[12]*0.01:.2f} Hz")
    else: p(f"  [E410-E41F] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE420,16)
    if s=="ok":
        p(f"  [E420] GridOFLevel2:       {r[0]*0.01:.2f} Hz")
        p(f"  [E424] ReConnectGridTime:  {r[4]} s")
        p(f"  [E425] IsoCheckEn:         {r[5]}")
        p(f"  [E42A] BattForGridPowerEn: {r[10]}")
        p(f"  [E42B] ExCtRatio:          {r[11]}")
        p(f"  [E42C] ZeroExportPower:    {r[12]} W")
    else: p(f"  [E420-E42F] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xE430,8)
    if s=="ok":
        p(f"  [E430-E437]: {[f'0x{v:04X}' for v in r]}")
    else: p(f"  [E430-E437] -> {s}")

def scan_p09(ser, addr):
    p(); p("="*65); p("  P09 - POWER STATISTICS (0xF000-0xF04D)"); p("="*65)
    labels=["yesterday","2d ago","3d ago","4d ago","5d ago","6d ago","7d ago"]
    r,s=read_regs(ser,addr,0xF000,28)
    if s=="ok":
        p("  PV energy last 7 days (x0.1 kWh):")
        for i,l in enumerate(labels): p(f"    [F{i:03X}] {l:10s}: {r[i]:3d} = {r[i]*0.1:.1f} kWh")
        p("  Bat charge last 7 days (Ah):")
        for i,l in enumerate(labels): p(f"    [F{7+i:03X}] {l:10s}: {r[7+i]} Ah")
        p("  Bat discharge last 7 days (Ah):")
        for i,l in enumerate(labels): p(f"    [F{14+i:03X}] {l:10s}: {r[14+i]} Ah")
        p("  Grid charge last 7 days (Ah):")
        for i,l in enumerate(labels): p(f"    [F{21+i:03X}] {l:10s}: {r[21+i]} Ah")
    else: p(f"  [F000-F01B] -> {s}")
    time.sleep(0.1)
    # F01C-F02B: Load + Grid consumption 7 days + date record
    r,s=read_regs(ser,addr,0xF01C,16)
    if s=="ok":
        p("  Load consumption last 7 days (x0.1 kWh):")
        for i,l in enumerate(labels): p(f"    [F{0x1C+i:03X}] {l:10s}: {r[i]*0.1:.1f} kWh")
        p("  Load from grid last 7 days (x0.1 kWh):")
        for i,l in enumerate(labels): p(f"    [F{0x23+i:03X}] {l:10s}: {r[7+i]*0.1:.1f} kWh")
        p(f"  [F02A-F02B] EnergyStatDay: raw {r[14]:#06x} {r[15]:#06x}")
    else: p(f"  [F01C-F02B] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xF02C,8)
    if s=="ok":
        p(f"  [F02C] PV->Grid today:     {r[0]*0.1:.1f} kWh")
        p(f"  [F02D] BatChg today:       {r[1]} Ah  *** BAT CHG TODAY ***")
        p(f"  [F02E] BatDchg today:      {r[2]} Ah  *** BAT DCHG TODAY ***")
        p(f"  [F02F] PV today:           {r[3]*0.1:.1f} kWh  *** PV TODAY ***")
        p(f"  [F030] Load today:         {r[4]*0.1:.1f} kWh  *** LOAD TODAY ***")
        p(f"  [F031] WorkDaysTotal:      {r[5]} days")
        p(f"  [F032-F033] GridEnergTot:  {u32(r[6],r[7])*0.1:.1f} kWh (32-bit)")
    else: p(f"  [F02C-F033] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xF034,8)
    if s=="ok":
        p(f"  [F034-F035] BatChgTotal:   {u32(r[0],r[1])} Ah  *** TOTAL CHG AH ***")
        p(f"  [F036-F037] BatDchgTotal:  {u32(r[2],r[3])} Ah  *** TOTAL DCHG AH ***")
        p(f"  [F038-F039] PV Total:      {u32(r[4],r[5])*0.1:.1f} kWh  *** PV CUMULATIVE ***")
        p(f"  [F03A-F03B] Load Total:    {u32(r[6],r[7])*0.1:.1f} kWh  *** LOAD CUMULATIVE ***")
    else: p(f"  [F034-F03B] -> {s}")
    time.sleep(0.1)
    r,s=read_regs(ser,addr,0xF03C,18)
    if s=="ok":
        p(f"  [F03C] GridChg today:      {r[0]} Ah")
        p(f"  [F03D] GridLoad today:     {r[1]*0.1:.1f} kWh")
        p(f"  [F03E] InvWork today:      {r[2]} min  *** INV WORK TODAY ***")
        p(f"  [F03F] GridWork today:     {r[3]} min")
        r0=r[4]; r1=r[5]; r2=r[6]
        try:
            yr=(r0>>8)+2002
            p(f"  [F040-F042] PowerOnTime:   {yr}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}")
        except: p(f"  [F040-F042] PowerOnTime:   raw {r0:#06x} {r1:#06x} {r2:#06x}")
        p(f"  [F046-F047] GridChgTotal:  {u32(r[10],r[11])} Ah")
        p(f"  [F048-F049] LoadLineTotal: {u32(r[12],r[13])*0.1:.1f} kWh")
        p(f"  [F04A] InvWorkTotal:       {r[14]} h  *** INV WORK TOTAL ***")
        p(f"  [F04B] GridWorkTotal:      {r[15]} h")
        p(f"  [F04C] GridChgKwH today:   {r[16]}")
    else: p(f"  [F03C-F04D] -> {s}")

def scan_p10(ser, addr):
    p(); p("="*65); p("  P10 - FAULT RECORDS (0xF800-0xF9F0, 32 records x 16 regs)"); p("="*65)
    p("  Format: [0]=fault_code  [1-3]=time(RTC)  [4-15]=data snapshot")
    p("  fault_code=0 means empty/invalid record")
    fault_count=0
    for rec in range(32):
        base=0xF800+rec*0x10
        r,s=read_regs(ser,addr,base,16)
        if s=="ok":
            fc=r[0]
            if fc==0:
                p(f"  [F{base:04X}] Record {rec:2d}: EMPTY")
            else:
                fault_count+=1
                r0=r[1]; r1=r[2]; r2=r[3]
                try:
                    yr=(r0>>8)+2002
                    t=f"{yr}-{r0&0xFF:02d}-{r1>>8:02d} {r1&0xFF:02d}:{r2>>8:02d}:{r2&0xFF:02d}"
                except: t=f"raw {r0:#06x} {r1:#06x} {r2:#06x}"
                fdesc=FAULT_CODES.get(fc,f"code {fc}")
                p(f"  [F{base:04X}] Record {rec:2d}: FAULT {fc} = {fdesc}")
                p(f"           Time: {t}")
                p(f"           Data: {[f'0x{v:04X}' for v in r[4:16]]}")
        else: p(f"  [F{base:04X}] Record {rec:2d} -> {s}")
        time.sleep(0.05)
    p(); p(f"  Total fault records: {fault_count}/32")

AREAS = {
    'p00':[scan_p00], 'p01':[scan_p01], 'p02':[scan_p02],
    'p03':[scan_p03], 'p05':[scan_p05], 'p07':[scan_p07],
    'p08':[scan_p08], 'p09':[scan_p09], 'p10':[scan_p10],
}
AREAS['all']=[scan_p00,scan_p01,scan_p02,scan_p03,scan_p05,scan_p07,scan_p08,scan_p09,scan_p10]

def main():
    setup_logger()
    parser=argparse.ArgumentParser(description="SRNE Full Scanner v2.0 - Protocol v1.96")
    parser.add_argument("port",nargs="?",default="/dev/ttyUSB1")
    parser.add_argument("--addr",type=lambda x:int(x,0),default=1)
    parser.add_argument("--baud",type=int,default=9600)
    parser.add_argument("--area",choices=list(AREAS.keys()),default="all")
    args=parser.parse_args()
    p("="*65)
    p("  SRNE Energy Storage Inverter - Complete Scanner v2.0")
    p("  Protocol: MODBUS Energy Storage Inverter v1.96 (Jan 2024)")
    p(f"  Log: {LOG_FILE}")
    p(f"  Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p("="*65)
    p(f"  Port: {args.port} | Baud: {args.baud} | Addr: {args.addr} | Area: {args.area}")
    try:
        ser=serial.Serial(args.port,args.baud,bytesize=8,parity=serial.PARITY_NONE,stopbits=1,timeout=0.1)
        p(f"  Port OK: {args.port}")
    except serial.SerialException as e:
        p(f"  FAIL: {e}"); sys.exit(1)
    time.sleep(0.3)
    for fn in AREAS[args.area]:
        fn(ser,args.addr); time.sleep(0.2)
    p(); p("="*65); p("  Scanare completa."); p(f"  Log: {LOG_FILE}"); p("="*65)
    ser.close()

if __name__=="__main__": main()
