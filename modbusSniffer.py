import struct
import paho.mqtt.client as mqtt
import json

# MQTT-Konfiguration
MQTT_BROKER = "192.168.1.149"
MQTT_PORT = 1882
MQTT_USERNAME = "user1"
MQTT_PASSWORD = "user1"
MQTT_TOPIC = "dtsu666/values"

# Register-Mapping basierend auf Doku (Adresse: Name)
REGISTER_MAP = {
    0x2004: "Uca",
    0x2006: "Ua",
    0x2008: "Ub",
    0x200A: "Uc",
    0x200C: "Ia",
    0x200E: "Ib",
    0x2010: "Ic",
    0x2012: "Pt",
    0x2014: "Pa",
    0x2016: "Pb",
    0x2018: "Pc",
    0x201A: "Qt",
    0x201C: "Qa",
    0x201E: "Qb",
    0x2020: "Qc",
    0x202A: "PFt",
    0x202C: "PFa",
    0x202E: "PFb",
    0x2030: "PFc",
    0x2044: "Freq",
    0x2050: "DmPt",
}

# Hier anpassen je nachdem wo der Dump beginnt
START_REGISTER = 0x2004

def parse_float32(data):
    try:
        return struct.unpack(">f", data)[0]
    except Exception:
        return None

def parse_frame(hex_data: str):
    data_bytes = bytes.fromhex(hex_data)
    values = {}
    total_registers = len(data_bytes) // 2
    for i in range(0, len(data_bytes), 4):
        reg = START_REGISTER + (i // 2)
        if reg in REGISTER_MAP:
            raw = data_bytes[i:i+4]
            value = parse_float32(raw)
            if value is not None:
                values[REGISTER_MAP[reg]] = round(value, 3)
    return values

def print_values(values):
    print("📨 Werte:")
    for name, value in values.items():
        print(f"  - {name}: {value}")

def send_mqtt(values):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.publish(MQTT_TOPIC, json.dumps(values))
    client.disconnect()
    print("✅ MQTT gesendet")

if __name__ == "__main__":
    hex_data = "451460004514300045149000439b00004599a00045873000c6ac6a00c3880000c6341800c620780045226000441f800044b5800043fd800046b0fe00443680004636000046209000c4734000c3ba0000c4774000c47a0000"
    values = parse_frame(hex_data)
    print_values(values)
    send_mqtt(values)

