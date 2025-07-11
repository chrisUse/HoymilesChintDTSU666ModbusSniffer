import serial
import struct
import time

# Konfiguration der seriellen Schnittstelle
ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

def calc_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 1):
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# Mapping von Float-Werten zu Feldnamen, Einheiten und Skalierungsfaktoren
FIELD_MAP = {
    0: ("Voltage L1", "V", 0.1),
    1: ("Voltage L2", "V", 0.1),
    2: ("Voltage L3", "V", 0.1),
    3: ("Current L1", "A", 0.01),
    4: ("Current L2", "A", 0.01),
    5: ("Current L3", "A", 0.01),
    6: ("Active Power Total", "W", 0.1),
    7: ("Reactive Power Total", "var", 0.1),
    8: ("Apparent Power Total", "VA", 0.1),
    9: ("Power Factor Total", "", 0.001),
    10: ("Frequency", "Hz", 0.01),
    11: ("Energy Import", "kWh", 0.01),
    12: ("Energy Export", "kWh", 0.01),
    13: ("Reactive Energy", "kvarh", 0.01),
    14: ("Max Voltage", "V", 0.1),
    15: ("Max Current", "A", 0.01),
    16: ("Voltage THD", "%", 0.01),
    17: ("Current THD", "%", 0.01),
    18: ("Unknown [18]", "", 1.0),
    19: ("Unknown [19]", "", 1.0),
    20: ("Unknown [20]", "", 1.0),
    21: ("Unknown [21]", "", 1.0),
}

print("📡 Sniffen auf /dev/ttyUSB0 @ 9600 Baud...")

while True:
    if ser.in_waiting >= 5:
        raw = ser.read(ser.in_waiting)
        for i in range(len(raw) - 5):
            if raw[i] == 0x9F and raw[i+1] == 0x03:  # Adresse 0x9F und Funktion 3
                length = raw[i+2]
                frame_end = i + 3 + length + 2
                if frame_end > len(raw):
                    continue
                frame = raw[i:frame_end]
                data = frame[:-2]
                crc_received = int.from_bytes(frame[-2:], byteorder='little')
                crc_calculated = calc_crc(data)
                if crc_received == crc_calculated:
                    print(f"\n📨 Gültiges Frame empfangen (Adresse {data[0]}):")
                    hex_str = data[3:].hex()
                    print(f"  ➤ Daten (Hex): {hex_str}")
                    print("  ➤ Float-Werte:")
                    for j in range(0, len(data[3:]), 4):
                        if j + 4 <= len(data[3:]):
                            val_bytes = data[3+j:3+j+4]
                            f = struct.unpack('>f', val_bytes)[0]
                            index = j // 4
                            label, unit, factor = FIELD_MAP.get(index, (f"Value {index}", "", 1.0))
                            print(f"    - {label}: {f * factor:.3f} {unit}")
                else:
                    print("❌ CRC-Fehler, verwerfe Frame.")
    time.sleep(0.1)

