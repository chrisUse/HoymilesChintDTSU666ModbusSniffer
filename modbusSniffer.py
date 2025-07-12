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

# Offset für den Start der Registerdaten im Payload (in Bytes)
REGISTER_PAYLOAD_OFFSET = 8  # Ab hier beginnen die plausiblen Werte

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

def extract_first_valid_modbus_frame(hex_string: str):
    """Durchsucht den Hex-String nach dem ersten gültigen Modbus-Frame und gibt (address, function_code, payload) zurück."""
    data = bytes.fromhex(hex_string)
    i = 0
    while i < len(data) - 5:
        address = data[i]
        function_code = data[i+1]
        byte_count = data[i+2]
        frame_len = 3 + byte_count + 2  # 3 Header + Daten + 2 CRC
        if i + frame_len <= len(data):
            payload = data[i+3:i+3+byte_count]
            # CRC = data[i+3+byte_count:i+3+byte_count+2]
            if len(payload) == byte_count:
                return address, function_code, payload
        i += 1
    return None, None, None

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

def debug_modbus_float_variants(chunk: bytes):
    """Gibt verschiedene Interpretationen eines 4-Byte-Chunks als Float aus."""
    if len(chunk) != 4:
        return
    variants = {
        'C D A B': chunk[2:4] + chunk[0:2],
        'A B C D': chunk[0:4],
        'B A D C': chunk[1::-1] + chunk[3:1:-1],
        'D C B A': chunk[::-1],
    }
    print(f"  Bytes: {chunk.hex()}")
    for name, b in variants.items():
        try:
            val = struct.unpack('>f', b)[0]
            print(f"    {name}: {b.hex()} = {val}")
        except Exception as e:
            print(f"    {name}: {b.hex()} = Fehler: {e}")

def parse_modbus_float_inverse(data: bytes):
    """Parst einen 4-Byte Chunk als Floating Inverse (z.B. für Power-Werte)."""
    if len(data) != 4:
        return None
    try:
        # Annahme: Die Umkehrung der Byte-Reihenfolge ergibt den korrekten Wert
        reversed_data = data[::-1]
        return struct.unpack(">f", reversed_data)[0]
    except Exception:
        return None

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
            address, function_code, payload = extract_first_valid_modbus_frame(hex_data)
            if payload is None:
                print("❌ Kein gültiger Modbus-Frame gefunden oder zu kurz!")
                continue

            print(f"Payload (hex): {payload.hex()}")
            print(f"Adresse: {address}, Funktionscode: {function_code}")

            # Nur Modbus-Read-Register (0x03, 0x04) verarbeiten
            if function_code not in (0x03, 0x04):
                print(f"⚠️  Funktionscode {function_code:#04x} wird ignoriert (kein Read-Register). Frame wird übersprungen.")
                continue

            # Extrahiere ab Byte 8 im Payload (offset=8)
            offset = 8

            # Prüfe, ob genug Daten für alle Werte vorhanden sind
            max_values = min(max((len(payload) - offset) // 4, 0), len(LABELS))
            if max_values < len(LABELS):
                print(f"⚠️  Warnung: Payload zu kurz für alle Werte! Es werden nur {max_values} von {len(LABELS)} extrahiert.")

            # Werte ab Offset extrahieren (jetzt als Big-Endian Float)
            values = []
            for idx in range(max_values):
                i = offset + idx * 4
                chunk = payload[i:i+4]
                if len(chunk) < 4:
                    break
                val = parse_float32_be(chunk)
                if val is not None:
                    values.append(round(val, 3))

            # Debug-Ausgabe: Anzahl und Inhalt der extrahierten Werte
            print(f"\n🧩 Extrahierte Werte ab Offset {offset} (insgesamt {len(values)} Werte):")
            for idx, v in enumerate(values[:10]):
                print(f"  [{idx}] = {v}")
            if len(values) > 10:
                print(f"  ... {len(values)-10} weitere Werte ...")
            print(f"Payload-Länge: {len(payload)} Bytes")

            # Debug: Zeige das gesamte Payload in 4-Byte-Chunks (ab Byte 0) als Floating Inverse und Big-Endian
            print(f"\n🔬 Debug: Alle 4-Byte-Chunks im Payload (ab Byte 0):")
            for i in range(0, len(payload) - 3, 4):
                chunk = payload[i:i+4]
                if len(chunk) < 4:
                    break
                fi = parse_modbus_float_inverse(chunk)
                be = parse_float32_be(chunk)
                print(f"  Bytes {i:02}-{i+3:02}: {chunk.hex()} | Floating Inverse: {fi} | Big-Endian: {be}")

            mapped_values = map_values_to_labels(values)

            print("📨 Werte:")
            for k, v in mapped_values.items():
                print(f"  - {k}: {v}")

            send_mqtt(mapped_values)

        except Exception as e:
            print(f"❌ Fehler: {e}")

        time.sleep(10)

