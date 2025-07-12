import struct
import paho.mqtt.client as mqtt
import json

# ✅ MQTT-Konfiguration
MQTT_BROKER = "192.168.1.149"
MQTT_PORT = 1882
MQTT_USERNAME = "user1"
MQTT_PASSWORD = "user1"
MQTT_TOPIC = "dtsu666/values"

# ✅ Register-Mapping gemäß Dokumentation
REGISTER_MAP = [
    ("Uca",  0x2004),
    ("Ua",   0x2006),
    ("Ub",   0x2008),
    ("Uc",   0x200A),
    ("Ia",   0x200C),
    ("Ib",   0x200E),
    ("Ic",   0x2010),
    ("Pt",   0x2012),
    ("Pa",   0x2014),
    ("Pb",   0x2016),
    ("Pc",   0x2018),
    ("Qt",   0x201A),
    ("Qa",   0x201C),
    ("Qb",   0x201E),
    ("Qc",   0x2020),
    ("PFt",  0x202A),
    ("PFa",  0x202C),
    ("PFb",  0x202E),
    ("PFc",  0x2030),
    ("Freq", 0x2044),
    ("DmPt", 0x2050),
]

BASE_REGISTER = 0x2004  # Startadresse im Modbus-Frame

# ✅ Float-Parsung: Standard-Bytefolge (ABCD)
def parse_float32(data):
    if len(data) != 4:
        return None
    try:
        return struct.unpack(">f", data)[0]  # keine Umordnung!
    except Exception:
        return None

# ✅ Frame-Auswertung
def parse_frame(hex_data: str):
    data_bytes = bytes.fromhex(hex_data)
    register_values = {}
    for name, reg in REGISTER_MAP:
        index = (reg - BASE_REGISTER) * 2
        if index + 4 <= len(data_bytes):
            raw = data_bytes[index:index+4]
            value = parse_float32(raw)
            if value is not None:
                register_values[name] = round(value, 3)
    return register_values

# ✅ Konsolenausgabe
def print_values(values):
    print("📨 Werte:")
    for name, value in values.items():
        print(f"  - {name}: {value}")

# ✅ MQTT-Publish
def send_mqtt(values):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.publish(MQTT_TOPIC, json.dumps(values))
    client.disconnect()
    print("✅ MQTT gesendet")

# ✅ Beispielverwendung
if __name__ == "__main__":
    # Beispiel-Frame (Hex-Daten aus passivem Mitschnitt)
    hex_data = "451460004514300045149000439b00004599a00045873000c6ac6a00c3880000c6341800c620780045226000441f800044b5800043fd800046b0fe00443680004636000046209000c4734000c3ba0000c4774000c47a0000"
    values = parse_frame(hex_data)
    print_values(values)
    send_mqtt(values)

