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
MQTT_USER = "user1"
MQTT_PASS = "user1"

# MQTT-Verbindung aufbauen
mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID, protocol=mqtt.MQTTv311)
mqtt_client.username_pw_set(username=MQTT_USER, password=MQTT_PASS)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

def calc_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def check_crc(frame: bytes) -> bool:
    data = frame[:-2]
    # Variante 1: CRC low byte zuerst
    received_crc = frame[-2] | (frame[-1] << 8)
    calculated_crc = calc_crc(data)
    if received_crc == calculated_crc:
        return True
    # Variante 2: CRC high byte zuerst
    received_crc = (frame[-2] << 8) | frame[-1]
    return received_crc == calculated_crc

def hex_to_float(hex_str):
    try:
        b = bytes.fromhex(hex_str)
        return struct.unpack(">f", b)[0]
    except Exception:
        return None

def parse_data(hex_data):
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
    floats = [hex_to_float(hex_data[i:i+8]) for i in range(0, len(hex_data), 8)]
    return dict(zip(keys, floats))

def main():
    print("📡 Sniffen auf /dev/ttyUSB0 @ 9600 Baud...")
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    while True:
        # Schritt 1: Header lesen (Adresse, Funktion, Länge)
        header = ser.read(3)
        if len(header) < 3:
            continue
        address, function, length = header

        # Nur Modbus Funktion 3 und Adresse 0x9F (159) verarbeiten
        if address != 0x9F or function != 0x03:
            # falls anderer Frame, kurz ignorieren
            ser.read(ser.in_waiting or 0)  # ggf. Clear Buffer
            continue

        # Schritt 2: Payload + CRC lesen
        payload_crc = ser.read(length + 2)
        if len(payload_crc) < length + 2:
            continue

        frame = header + payload_crc

        if not check_crc(frame):
            print("❌ CRC-Fehler, verwerfe Frame.")
            continue

        # Payload (ohne Adresse, Funktion, Länge und CRC)
        data_payload = frame[3:-2]
        data_hex = binascii.hexlify(data_payload).decode()

        print(f"📨 Gültiges Frame empfangen (Adresse {address}):")
        print(f"  ➤ Daten (Hex): {data_hex}")

        values = parse_data(data_hex)

        print("  ➤ Float-Werte:")
        for k, v in values.items():
            print(f"    - {k}: {v:.3f}")

        # MQTT senden
        mqtt_payload = json.dumps(values)
        mqtt_client.publish(MQTT_TOPIC, mqtt_payload)
        print(f"  ✅ MQTT gesendet an '{MQTT_TOPIC}'")

        time.sleep(1)

if __name__ == "__main__":
    main()

