import serial
import struct
import binascii
import time
import json
import paho.mqtt.client as mqtt

# MQTT-Konfiguration
MQTT_BROKER = "192.168.1.149"
MQTT_PORT = 1882
MQTT_TOPIC = "dtsu666/values"
MQTT_CLIENT_ID = "dtsu666_sniffer"

# MQTT-Verbindung aufbauen
mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
mqtt_client.username_pw_set(username="user1", password="user1")
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

def hex_to_float(hex_str):
    try:
        b = bytes.fromhex(hex_str)
        return struct.unpack(">f", b)[0]
    except Exception:
        return None

def parse_data(hex_data):
    # Frame besteht aus 22 Float-Werten, also 22 * 4 = 88 Bytes = 176 hex chars
    floats = [hex_to_float(hex_data[i:i+8]) for i in range(0, len(hex_data), 8)]

    # Benenne die wichtigsten Werte
    keys = [
        "Voltage L1", "Voltage L2", "Voltage L3",
        "Current L1", "Current L2", "Current L3",
        "Active Power Total", "Reactive Power Total",
        "Apparent Power Total", "Power Factor Total",
        "Frequency", "Energy Import", "Energy Export", "Reactive Energy",
        "Max Voltage", "Max Current",
        "Voltage THD", "Current THD",
        "Unknown 18", "Unknown 19", "Unknown 20", "Unknown 21"
    ]

    return dict(zip(keys, floats))

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

def check_crc(frame: bytes) -> bool:
    if len(frame) < 5:
        return False
    data = frame[:-2]
    received_crc = frame[-2] | (frame[-1] << 8)
    calculated_crc = calc_crc(data)
    return received_crc == calculated_crc

def main():
    print("📡 Sniffen auf /dev/ttyUSB0 @ 9600 Baud...")
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    while True:
        raw = ser.read(300)
        if len(raw) < 10:
            continue

        if not check_crc(raw):
            print("❌ CRC-Fehler, verwerfe Frame.")
            continue

        hex_data = binascii.hexlify(raw).decode()

        # Adresse 159, Funktion 3 = ab Byte 3
        payload_start = 3
        payload_length = len(raw) - 5  # ohne Adresse, Funktion, CRC
        data_payload = raw[payload_start:payload_start + payload_length]
        data_hex = binascii.hexlify(data_payload).decode()

        print(f"📨 Gültiges Frame empfangen (Adresse {raw[0]}):")
        print(f"  ➤ Daten (Hex): {data_hex}")

        values = parse_data(data_hex)

        print("  ➤ Float-Werte:")
        for k, v in values.items():
            print(f"    - {k}: {v:.3f}")

        # 📤 MQTT-Publish
        mqtt_payload = json.dumps(values)
        mqtt_client.publish(MQTT_TOPIC, mqtt_payload)
        print(f"  ✅ MQTT gesendet an '{MQTT_TOPIC}'")

        time.sleep(1)

if __name__ == "__main__":
    main()

