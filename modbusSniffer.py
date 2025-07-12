import serial, struct, binascii, time, json, paho.mqtt.client as mqtt

# MQTT-Konfiguration
MQTT_BROKER = "192.168.1.149"
MQTT_PORT = 1882
MQTT_TOPIC = "dtsu666/values"
MQTT_CLIENT_ID = "dtsu666_sniffer"
MQTT_USER = "user1"
MQTT_PASS = "user1"

mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID, protocol=mqtt.MQTTv311)
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Register-Mapping (Adresse = Modbus-Offset ab 0x2004)
REGISTER_MAP = [
    ("Uca (Phase-to-neutral Voltage)", 0x2004),
    ("Ua (Phase-to-phase Voltage A-B)", 0x2006),
    ("Ub (Phase-to-phase Voltage B-C)", 0x2008),
    ("Uc (Phase-to-phase Voltage C-A)", 0x200A),
    ("Ia (Phase current A)", 0x200C),
    ("Ib (Phase current B)", 0x200E),
    ("Ic (Phase current C)", 0x2010),
    ("Pt (Total Active Power)", 0x2012),
    ("Pa (Phase A Active Power)", 0x2014),
    ("Pb (Phase B Active Power)", 0x2016),
    ("Pc (Phase C Active Power)", 0x2018),
    ("Qt (Total Reactive Power)", 0x201A),
    ("Qa (Phase A Reactive Power)", 0x201C),
    ("Qb (Phase B Reactive Power)", 0x201E),
    ("Qc (Phase C Reactive Power)", 0x2020),
    ("PFt (Total Power Factor)", 0x202A),
    ("PFa (Phase A Power Factor)", 0x202C),
    ("PFb (Phase B Power Factor)", 0x202E),
    ("PFc (Phase C Power Factor)", 0x2030),
    ("Freq (Frequency)", 0x2044),
    ("DmPt (Demand Active Power)", 0x2050),
]

def calc_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc

def check_crc(frame: bytes) -> bool:
    data, rec_lo, rec_hi = frame[:-2], frame[-2], frame[-1]
    received = rec_lo | (rec_hi << 8)
    return received == calc_crc(data) or received == ((rec_hi << 8) | rec_lo)

def read_float_be(b: bytes) -> float:
    return struct.unpack(">f", b)[0]

def main():
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
    print("📡 Sniffen auf /dev/ttyUSB0 @ 9600 Baud...")

    while True:
        header = ser.read(3)
        if len(header) < 3: continue
        addr, func, length = header
        if addr != 0x9F or func != 0x03:
            ser.read(ser.in_waiting or 0)
            continue
        body = ser.read(length + 2)
        if len(body) < length + 2: continue
        frame = header + body
        if not check_crc(frame):
            print("❌ CRC-Fehler, verwerfe Frame.")
            continue

        payload = frame[3:-2]
        values = {}
        for name, reg_addr in REGISTER_MAP:
            idx = (reg_addr - 0x2004) * 2  # Byte-Offset im Payload
            if idx+4 <= len(payload):
                try:
                    values[name] = round(read_float_be(payload[idx:idx+4]), 3)
                except:
                    values[name] = None
            else:
                values[name] = None

        print("📨 Werte:")
        for n, v in values.items():
            print(f"  - {n}: {v}")

        mqtt_client.publish(MQTT_TOPIC, json.dumps(values))
        print("✅ MQTT gesendet")
        time.sleep(1)

if __name__ == "__main__":
    main()

