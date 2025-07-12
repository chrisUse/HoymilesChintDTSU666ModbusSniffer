import struct
import paho.mqtt.client as mqtt
import json
import time
import serial  # pyserial, install: pip install pyserial

# MQTT-Konfiguration
MQTT_BROKER = "192.168.1.149"
MQTT_PORT = 1882
MQTT_USERNAME = "user1"
MQTT_PASSWORD = "user1"
MQTT_TOPIC = "dtsu666/values"

# Register und Labels (die Reihenfolge muss zu den Werten passen)
LABELS = [
    "Uca", "Ua", "Ub", "Uc", "Ia", "Ib", "Ic",
    "Pt", "Pa", "Pb", "Pc", "Qt", "Qa", "Qb",
    "Qc", "PFt", "PFa", "PFb", "PFc", "Freq", "DmPt"
]

def parse_float32_be(data: bytes):
    """Big-Endian Float aus 4 Bytes."""
    if len(data) != 4:
        return None
    try:
        return struct.unpack(">f", data)[0]
    except Exception:
        return None

def parse_sniffer_hex(hex_string: str):
    """Parst Hex-String vom Sniffer und gibt Float-Werte-Liste zurück."""
    data = bytes.fromhex(hex_string)
    floats = []
    for i in range(0, len(data), 4):
        chunk = data[i:i+4]
        if len(chunk) < 4:
            break
        val = parse_float32_be(chunk)
        if val is not None:
            floats.append(round(val, 3))
    return floats

def parse_modbus_rtu_frame(hex_string: str):
    """Parst einen Modbus RTU Frame aus einem Hex-String und gibt die Nutzdaten zurück."""
    data = bytes.fromhex(hex_string)
    if len(data) < 5:
        return None, None, None  # Zu kurz
    address = data[0]
    function_code = data[1]
    # Annahme: Byte 2 ist Byteanzahl der Daten
    byte_count = data[2]
    if len(data) < 3 + byte_count + 2:
        return None, None, None  # Frame zu kurz für angegebene Datenlänge
    payload = data[3:3+byte_count]
    # CRC = data[-2:]
    return address, function_code, payload

def map_values_to_labels(values):
    """Mappt eine Liste von Werten auf Labels."""
    result = {}
    for i, label in enumerate(LABELS):
        if i < len(values):
            result[label] = values[i]
    return result

def send_mqtt(values):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.publish(MQTT_TOPIC, json.dumps(values))
    client.disconnect()
    print("✅ MQTT gesendet")

def read_from_serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1):
    """Liest eine Zeile Hex-Daten von serieller Schnittstelle (ohne UTF-8-Dekodierung)."""
    with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
        line = ser.readline()
        # Gib Hex-String zurück, entferne Whitespaces
        return line.hex().strip()

if __name__ == "__main__":
    serial_port = "/dev/ttyUSB0"  # passe ggf. an
    baudrate = 9600               # passe ggf. an

    while True:
        try:
            print("📡 Warte auf Daten vom Zähler...")
            hex_data = read_from_serial(serial_port, baudrate)
            if not hex_data:
                continue

            print(f"Hex-Daten vom Sniffer: {hex_data}")
            address, function_code, payload = parse_modbus_rtu_frame(hex_data)
            if payload is None:
                print("❌ Ungültiger Modbus-Frame oder zu kurz!")
                continue

            print(f"Adresse: {address}, Funktionscode: {function_code}")
            values = []
            for i in range(0, len(payload), 4):
                chunk = payload[i:i+4]
                if len(chunk) < 4:
                    break
                val = parse_float32_be(chunk)
                if val is not None:
                    values.append(round(val, 3))
            mapped_values = map_values_to_labels(values)

            print("📨 Werte:")
            for k, v in mapped_values.items():
                print(f"  - {k}: {v}")

            send_mqtt(mapped_values)

        except Exception as e:
            print(f"❌ Fehler: {e}")

        time.sleep(10)

