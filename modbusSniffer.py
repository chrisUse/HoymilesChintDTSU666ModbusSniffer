import struct
import paho.mqtt.client as mqtt
import json
import time
import math
import serial  # pyserial, install: pip install pyserial

# MQTT-Konfiguration
MQTT_BROKER = "192.168.1.149"
MQTT_PORT = 1882
MQTT_USERNAME = "user1"
MQTT_PASSWORD = "user1"
MQTT_TOPIC = "dtsu666/values"

# Register und Labels mit Registeradressen
REGISTER_MAP = {
    0x2000: "Uab", 0x2002: "Ubc", 0x2004: "Uca", 0x2006: "Ua", 0x2008: "Ub", 0x200A: "Uc", 
    0x200C: "Ia", 0x200E: "Ib", 0x2010: "Ic",
    0x2012: "Pt", 0x2014: "Pa", 0x2016: "Pb", 0x2018: "Pc", 
    0x201A: "Qt", 0x201C: "Qa", 0x201E: "Qb",
    0x2020: "Qc", 0x202A: "PFt", 0x202C: "PFa", 0x202E: "PFb", 0x2030: "PFc", 
    0x2044: "Freq"
}

# Liste aller Labels in der Reihenfolge der Registernummern
LABELS = [
    "Uca", "Ua", "Ub", "Uc", "Ia", "Ib", "Ic",
    "Pt", "Pa", "Pb", "Pc", "Qt", "Qa", "Qb",
    "Qc", "PFt", "PFa", "PFb", "PFc", "Freq", "DmPt"
]

# Skalierungsfaktoren für verschiedene Messwerte (basierend auf der DTSU666-Dokumentation)
SCALING_FACTORS = {
    # Spannungen (V): Direktwert ohne Skalierung
    "Uca": 0.1, "Ua": 0.1, "Ub": 0.1, "Uc": 0.1,
    # Ströme (A): Direktwert ohne Skalierung
    "Ia": 1.0, "Ib": 1.0, "Ic": 1.0,
    # Leistungswerte (W): Direktwert ohne Skalierung
    "Pt": 1.0, "Pa": 1.0, "Pb": 1.0, "Pc": 1.0,
    # Blindleistung (var): Direktwert ohne Skalierung
    "Qt": 1.0, "Qa": 1.0, "Qb": 1.0, "Qc": 1.0,
    # Leistungsfaktoren: Direktwert ohne Skalierung
    "PFt": 1.0, "PFa": 1.0, "PFb": 1.0, "PFc": 1.0,
    # Frequenz (Hz): Direktwert ohne Skalierung
    "Freq": 1.0,
    # Demand (W): Direktwert ohne Skalierung
    "DmPt": 1.0
}

# Einheiten für die verschiedenen Messwerte
UNITS = {
    "Uca": "V", "Ua": "V", "Ub": "V", "Uc": "V",
    "Ia": "A", "Ib": "A", "Ic": "A",
    "Pt": "W", "Pa": "W", "Pb": "W", "Pc": "W",
    "Qt": "var", "Qa": "var", "Qb": "var", "Qc": "var",
    "PFt": "", "PFa": "", "PFb": "", "PFc": "",
    "Freq": "Hz",
    "DmPt": "W"
}


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
    """Durchsucht den Hex-String nach dem ersten gültigen Modbus-Frame und gibt (address, function_code, payload) zurück.
    Erkennt Requests (Adresse 0x9f) und Responses (andere Adresse, Funktionscode 0x03/0x04)."""
    data = bytes.fromhex(hex_string)
    i = 0
    while i < len(data) - 5:
        address = data[i]
        function_code = data[i+1]
        
        # Request: Adresse 0x9f (Master), Funktionscode 0x03/0x04
        if address == 0x9f and function_code in (0x03, 0x04):
            if i + 8 <= len(data):
                payload = data[i+2:i+6]
                crc = data[i+6:i+8]
                return address, function_code, payload
                
        # Response: andere Adresse (z.B. 0x01), Funktionscode 0x03/0x04
        elif address != 0x9f and function_code in (0x03, 0x04):
            if i + 3 < len(data):
                byte_count = data[i+2]
                frame_len = 3 + byte_count + 2  # Adresse + Funktionscode + Byte-Count + Payload + 2 CRC-Bytes
                
                # Prüfe, ob genug Bytes für einen vollständigen Frame vorhanden sind
                if 1 <= byte_count <= 128 and i + frame_len <= len(data):
                    payload = data[i+3:i+3+byte_count]
                    # Prüfe, ob die Payload-Länge mit dem Byte-Count übereinstimmt
                    if len(payload) == byte_count:
                        return address, function_code, payload
        
        # Wenn kein Treffer, weiter zum nächsten Byte
        i += 1
        
    # Suche nach der nächsten vollständigen Antwort nach einer Anfrage
    # Typische Muster: 0x9F 0x03 <register> <count> ... dann folgt eine Antwort von 0x01 0x03 <byte_count> <data>
    i = 0
    # Suche zuerst einen Request
    while i < len(data) - 8:
        if data[i] == 0x9f and data[i+1] in (0x03, 0x04):
            # Potentieller Request gefunden
            request_start = i
            startreg = int.from_bytes(data[i+2:i+4], byteorder='big')
            regcount = int.from_bytes(data[i+4:i+6], byteorder='big')
            
            # Jetzt suche nach der Antwort danach
            resp_start = request_start + 8  # Nach dem Request
            while resp_start < len(data) - 5:
                if data[resp_start] != 0x9f and data[resp_start+1] in (0x03, 0x04):
                    # Potentielle Antwort gefunden
                    byte_count = data[resp_start+2]
                    expected_data_length = regcount * 4  # Float-Werte sind 4 Bytes pro Register
                    
                    # Prüfe, ob die erwartete Datenlänge mit dem Byte-Count übereinstimmt
                    if byte_count == expected_data_length and resp_start + 3 + byte_count <= len(data):
                        return data[resp_start], data[resp_start+1], data[resp_start+3:resp_start+3+byte_count]
                
                resp_start += 1
        
        i += 1
    
    # Spezielle Erkennung für DTSU666-Antworten, wenn kein vollständiges Modbus-Frame gefunden wurde
    # Suche nach Blöcken mit spezifischer Länge (44 Register = 88 Bytes für Standard-Register oder 176 Bytes für Extended-Register)
    for expected_length in [88, 176]:
        i = 0
        while i <= len(data) - expected_length:
            # Prüfe, ob wir einen Block von gültigen Float-Werten haben könnten
            valid_floats = 0
            total_chunks = expected_length // 4
            for j in range(0, expected_length, 4):
                chunk = data[i+j:i+j+4]
                if len(chunk) < 4:
                    break
                
                # Überprüfe die ersten 5 Register auf typische Werte für Spannung und Strom
                # DTSU666 liefert typischerweise Spannungswerte zwischen 0-500V und Stromwerte zwischen 0-100A
                try:
                    # Verwende das Floating Inverse Format (CD AB)
                    reordered_data = chunk[2:4] + chunk[0:2]
                    val = struct.unpack(">f", reordered_data)[0]
                    
                    # Sehr grobe Plausibilitätsprüfung für die ersten Register (Spannung, Strom, Leistung)
                    if not math.isnan(val) and not math.isinf(val) and -1e6 < val < 1e6:
                        valid_floats += 1
                        
                        # Für die ersten Werte (Spannung) erwarten wir 0-500V
                        if j < 16 and (0 <= val <= 500 or -0.1 <= val <= 0.1):  # Spannung oder 0
                            valid_floats += 1
                        # Für die nächsten Werte (Strom) erwarten wir 0-100A
                        elif 16 <= j < 28 and (0 <= val <= 100 or -0.1 <= val <= 0.1):  # Strom oder 0
                            valid_floats += 1
                except:
                    pass
            
            # Wenn wir genügend gültige Float-Werte haben (mindestens 20% der Chunks), behandle es als Response
            min_valid_floats = total_chunks * 0.2
            if valid_floats >= min_valid_floats:
                # Fiktive Werte für die Erkennung als Response
                return 1, 0x03, data[i:i+expected_length]
            
            i += 1
    
    return None, None, None

def map_values_to_labels(values, start_register=0x2000):
    """Mappt eine Liste von Werten auf Labels basierend auf dem Startregister.
    
    Args:
        values: Liste der zu mappenden Werte
        start_register: Das Startregister, das von der Modbus-Anfrage angefordert wurde (default: 0x2000)
    
    Returns:
        Ein Dictionary mit Label/Wert-Paaren
    """
    result = {}
    # Berechne, welche Labels für die Antwort verwendet werden sollen
    start_index = 0
    
    # Finde den Index des Startregisters in der REGISTER_MAP
    if start_register in REGISTER_MAP:
        # Wenn das Startregister direkt in der Map ist, finde das entsprechende Label
        start_label = REGISTER_MAP[start_register]
        # Finde den Index dieses Labels in der LABELS-Liste
        if start_label in LABELS:
            start_index = LABELS.index(start_label)
    
    # Debug-Ausgabe für das Mapping
    print(f"Mapping: Startregister=0x{start_register:04X}, Startindex={start_index}")
    
    # Wende das Mapping an, beginnend vom berechneten Startindex
    for i, value in enumerate(values):
        if start_index + i < len(LABELS):
            label = LABELS[start_index + i]
            # Wende Skalierungsfaktor an, falls vorhanden
            scaling_factor = SCALING_FACTORS.get(label, 1.0)
            # Runde auf 3 Nachkommastellen nach der Skalierung
            result[label] = round(value * scaling_factor, 3)
        else:
            # Wenn mehr Werte als Labels vorhanden sind, verwende generische Namen
            result[f"Register{start_register + i*2:04X}"] = round(value, 3)
    
    return result

def apply_plausibility_check(values_dict):
    """Prüft, ob die Werte physikalisch plausibel sind und ersetzt unplausible Werte durch 0."""
    plausibility_ranges = {
        # Spannung (V): 0-500V (typisch 230V-400V)
        "Uca": (0, 500), "Ua": (0, 500), "Ub": (0, 500), "Uc": (0, 500),
        # Strom (A): 0-100A (basierend auf Messbereich des DTSU666)
        "Ia": (0, 100), "Ib": (0, 100), "Ic": (0, 100),
        # Leistung (W): -50000 bis 50000W
        "Pt": (-50000, 50000), "Pa": (-50000, 50000), "Pb": (-50000, 50000), "Pc": (-50000, 50000),
        # Blindleistung (var): -50000 bis 50000var
        "Qt": (-50000, 50000), "Qa": (-50000, 50000), "Qb": (-50000, 50000), "Qc": (-50000, 50000),
        # Leistungsfaktor: -1 bis 1
        "PFt": (-1, 1), "PFa": (-1, 1), "PFb": (-1, 1), "PFc": (-1, 1),
        # Frequenz (Hz): 45-65Hz (typisch 50Hz oder 60Hz)
        "Freq": (45, 65),
        # Demand (W): 0-50000W
        "DmPt": (0, 50000)
    }
    
    result = {}
    for key, value in values_dict.items():
        min_val, max_val = plausibility_ranges.get(key, (-float('inf'), float('inf')))
        if min_val <= value <= max_val:
            result[key] = value
        else:
            # Wert außerhalb des plausiblen Bereichs
            result[key] = 0.0
            print(f"⚠️  Unplausible: {key}={value} {UNITS.get(key, '')} (Bereich: {min_val} bis {max_val} {UNITS.get(key, '')})")
    
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
        'C D A B (Float Inverse)': chunk[2:4] + chunk[0:2],
        'A B C D (Big-Endian)': chunk[0:4],
        'B A D C (Mixed-Endian)': chunk[1::-1] + chunk[3:1:-1],
        'D C B A (Little-Endian)': chunk[::-1],
    }
    print(f"  Bytes: {chunk.hex()}")
    for name, b in variants.items():
        try:
            val = struct.unpack('>f', b)[0]
            plausible = ""
            if not math.isnan(val) and not math.isinf(val):
                if -1000 < val < 1000:
                    # Typische Werte für Spannung (0-500V), Strom (0-100A), Frequenz (45-65Hz), etc.
                    if 0 <= val <= 500:
                        plausible = " ✅ (plausibel für Spannung)"
                    elif 0 <= val <= 100:
                        plausible = " ✅ (plausibel für Strom)"
                    elif 45 <= val <= 65:
                        plausible = " ✅ (plausibel für Frequenz)"
                    elif -1 <= val <= 1:
                        plausible = " ✅ (plausibel für Leistungsfaktor)"
                    elif -50000 <= val <= 50000:
                        plausible = " ✅ (plausibel für Leistung)"
                    else:
                        plausible = " ✅ (plausibel)"
                else:
                    plausible = " ❌ (unplausibel)"
            else:
                plausible = " ❌ (ungültig)"
                
            print(f"    {name}: {b.hex()} = {val}{plausible}")
        except Exception as e:
            print(f"    {name}: {b.hex()} = Fehler: {e}")
            
    # Versuche auch als Integer-Werte zu interpretieren
    try:
        int_be = int.from_bytes(chunk, byteorder='big', signed=True)
        int_le = int.from_bytes(chunk, byteorder='little', signed=True)
        uint_be = int.from_bytes(chunk, byteorder='big', signed=False)
        uint_le = int.from_bytes(chunk, byteorder='little', signed=False)
        print(f"  Integer-Interpretationen:")
        print(f"    Int32 Big-Endian: {int_be}")
        print(f"    Int32 Little-Endian: {int_le}")
        print(f"    UInt32 Big-Endian: {uint_be}")
        print(f"    UInt32 Little-Endian: {uint_le}")
    except Exception as e:
        print(f"    Integer-Dekodierung: Fehler: {e}")

def process_modbus_payload(payload: bytes, debug=False):
    """Verarbeitet einen Modbus-Payload und extrahiert Float-Werte."""
    if len(payload) < 4:
        return []
        
    values = []
    # Ermittle, wie viele 4-Byte-Blöcke wir im Payload haben
    max_values = len(payload) // 4
    
    # Debug: Zeige die ersten Bytes im Hex-Format
    if debug:
        print(f"Payload (hex): {payload.hex()}")
        print(f"Payload-Länge: {len(payload)} Bytes, erwartete Werte: {max_values}")
    
    for i in range(max_values):
        offset = i * 4
        chunk = payload[offset:offset+4]
        
        if len(chunk) != 4:
            continue
            
        # Laut DTSU666-Dokumentation: Float Inverse Format (CDAB)
        value = parse_modbus_float_inverse(chunk)
        
        if value is None or math.isnan(value) or math.isinf(value):
            if debug:
                print(f"Ungültiger Wert an Position {i}: Bytes={chunk.hex()}")
                debug_modbus_float_variants(chunk)
            values.append(0.0)  # Ersetze ungültige Werte durch 0
        else:
            values.append(round(value, 3))
            
    return values

def parse_modbus_float_inverse(data: bytes, offset=0):
    """Parst einen 4-Byte Chunk als Floating Inverse Format (AB CD).
    Laut Registertabelle des DTSU666 verwenden alle Register dieses Format."""
    if isinstance(data, bytes) and offset == 0 and len(data) == 4:
        # Direkter 4-Byte Chunk wurde übergeben
        chunk = data
    elif isinstance(data, bytes) and offset + 4 <= len(data):
        # Offset in einem größeren Byte-Array
        chunk = data[offset:offset+4]
    else:
        return None
        
    try:
        # Laut Registertabelle: Floating Inverse (AB CD) Format
        # Dies bedeutet wahrscheinlich eine Umordnung der Bytes:
        # Von [A, B, C, D] zu [C, D, A, B]
        # d.h. Vertauschung der beiden Wort-Hälften
        reordered_data = chunk[2:4] + chunk[0:2]
        result = struct.unpack(">f", reordered_data)[0]
        
        # Prüfe auf ungültige Werte (NaN, Inf, extrem große Werte)
        if not (-1e10 < result < 1e10) or math.isnan(result) or math.isinf(result):
            return None
            
        # Debugausgabe für sehr kleine Werte nahe Null, die möglicherweise Rundungsfehler sind
        if -1e-10 < result < 1e-10 and result != 0:
            result = 0.0  # Werte sehr nahe Null auf exakt Null setzen
            
        return result
    except Exception as e:
        print(f"Error parsing float: {e}")
        print(f"Bytes: {' '.join([f'{b:02x}' for b in chunk])}")
        return None

if __name__ == "__main__":
    serial_port = "/dev/ttyUSB0"  # passe ggf. an
    baudrate = 9600               # passe ggf. an

    while True:
        try:
            print("\U0001F4E1 Warte auf Daten vom Zähler...")
            hex_data = read_from_serial(serial_port, baudrate)
            if not hex_data:
                continue

            # NEU: Zeige jeden empfangenen Hex-String, auch die ganz kurzen
            print(f"[RAW] Empfangener Hex-String: {hex_data} (Länge: {len(hex_data)//2} Bytes)")

            # Verarbeite ALLE Frames im Hex-String nacheinander
            rest_hex = hex_data
            while rest_hex:
                address, function_code, payload = extract_first_valid_modbus_frame(rest_hex)
                if address is None or function_code is None or payload is None:
                    break  # Keine weiteren Frames gefunden

                print(f"Frame-Adresse: {address}, Funktionscode: {function_code:#04x}, Payload (hex): {payload.hex()}")

                # Unterscheide zwischen Request und Response anhand der Adresse und Payload-Länge
                if function_code in (0x03, 0x04):
                    # Request: Adresse 0x9f (Master)
                    if address == 0x9f and len(payload) == 4:
                        startreg = int.from_bytes(payload[0:2], byteorder='big')
                        regcount = int.from_bytes(payload[2:4], byteorder='big')
                        print(f"➡️  Modbus-Request erkannt: Adresse={address}, Funktion={function_code:#04x}, Startregister=0x{payload[0:2].hex()} ({startreg}), Registeranzahl=0x{payload[2:4].hex()} ({regcount})")
                        last_request = {'address': address, 'function_code': function_code, 'startreg': startreg, 'regcount': regcount, 'timestamp': time.time()}
                        # Frame-Länge für Request: 1+1+4+2=8 Bytes
                        frame_len = 8
                    # Spezielle Behandlung für rohe Payload-Daten (ohne Modbus-Framing)
                    elif len(payload) >= 88:  # Mindestens 22 Register (88 Bytes)
                        print(f"⬅️  Wahrscheinliche Modbus-Response (Rohdaten) erkannt: Datenlänge={len(payload)}")
                        # Wir haben hier nur die rohen Daten ohne Modbus-Framing
                        # Behandle die gesamte Payload als Registerdaten
                        offset = 0  # Kein Offset, da wir die Rohdaten haben
                        max_values = min(len(payload) // 4, len(LABELS))
                        if max_values < len(LABELS):
                            print(f"⚠️  Warnung: Payload enthält nur {max_values} von {len(LABELS)} erwarteten Werten!")
                        values = []
                        for idx in range(max_values):
                            i = offset + idx * 4
                            chunk = payload[i:i+4]
                            if len(chunk) < 4:
                                break
                            # Verwende Floating Inverse Format laut Registertabelle (AB CD)
                            val = parse_modbus_float_inverse(chunk)
                            if val is not None and not math.isnan(val) and not math.isinf(val) and -1e6 < val < 1e6:
                                values.append(round(val, 3))
                            else:
                                # Wenn ungültiger Wert, versuche mit 0.0 zu ersetzen
                                values.append(0.0)
                        print(f"\n🧩 Extrahierte Werte (insgesamt {len(values)} Werte):")
                        for idx, v in enumerate(values[:10]):
                            print(f"  [{idx}] = {v}")
                        if len(values) > 10:
                            print(f"  ... {len(values)-10} weitere Werte ...")
                        print(f"Payload-Länge: {len(payload)} Bytes")
                        print(f"\n🔬 Debug: Alle 4-Byte-Chunks im Payload (ab Byte 0):")
                        for i in range(0, min(len(payload), 40) - 3, 4):  # Begrenze auf die ersten 40 Bytes
                            chunk = payload[i:i+4]
                            if len(chunk) < 4:
                                break
                            fi = parse_modbus_float_inverse(chunk)
                            be = parse_float32_be(chunk)
                            print(f"  Bytes {i:02}-{i+3:02}: {chunk.hex()} | Floating Inverse: {fi} | Big-Endian: {be}")
                        
                        # Verwende 0x2000 als Standard-Startregister, falls kein letzter Request bekannt ist
                        startreg = 0x2000
                        if 'last_request' in locals() and 'startreg' in last_request:
                            startreg = last_request['startreg']
                        
                        mapped_values = map_values_to_labels(values, startreg)
                        # Wende Plausibilitätsprüfung an
                        mapped_values = apply_plausibility_check(mapped_values)
                        print("📨 Werte (mit Plausibilitätsprüfung):")
                        for k, v in mapped_values.items():
                            unit = UNITS.get(k, "")
                            print(f"  - {k}: {v} {unit}")
                        send_mqtt(mapped_values)
                        # Frame-Länge: Gesamte Payload plus ein Byte für den Übergang zum nächsten Frame
                        frame_len = len(payload) + 1
                    # Response: Single-Register (2 oder 4 Bytes) bei anderer Adresse als 0x9f
                    elif address != 0x9f and len(payload) in (2, 4):
                        reg_addr = None
                        if 'last_request' in locals() and last_request['function_code'] == function_code:
                            reg_addr = last_request['startreg']
                        if len(payload) == 2:
                            val = int.from_bytes(payload, byteorder='big')
                            print(f"⬅️  Einzel-Response: Adresse={address}, Funktion={function_code:#04x}, Register={reg_addr}, Wert (int)={val}")
                        else:
                            # Register values use Floating Inverse format according to documentation
                            val = parse_modbus_float_inverse(payload)
                            print(f"⬅️  Einzel-Response: Adresse={address}, Funktion={function_code:#04x}, Register={reg_addr}, Wert (float)={val}")
                        # Frame-Länge für Einzel-Response: 1+1+len(payload)+2
                        frame_len = 1+1+len(payload)+2
                    # Response: Multi-Register bei anderer Adresse als 0x9f mit mindestens 8 Bytes
                    elif address != 0x9f and len(payload) >= 8:
                        print(f"⬅️  Standard Modbus-Response erkannt: Adresse={address}, Funktion={function_code:#04x}, Datenlänge={len(payload)}")
                        # Keine Offset-Anpassung für Standard Modbus-Antworten nötig
                        offset = 0
                        max_values = min(len(payload) // 4, len(LABELS))
                        if max_values < len(LABELS):
                            print(f"⚠️  Warnung: Payload enthält nur {max_values} von {len(LABELS)} erwarteten Werten!")
                        values = []
                        for idx in range(max_values):
                            i = offset + idx * 4
                            chunk = payload[i:i+4]
                            if len(chunk) < 4:
                                break
                            # Verwende Floating Inverse Format laut Registertabelle (AB CD)
                            val = parse_modbus_float_inverse(chunk)
                            if val is not None:
                                values.append(round(val, 3))
                        print(f"\n🧩 Extrahierte Werte (insgesamt {len(values)} Werte):")
                        for idx, v in enumerate(values[:10]):
                            print(f"  [{idx}] = {v}")
                        if len(values) > 10:
                            print(f"  ... {len(values)-10} weitere Werte ...")
                        print(f"Payload-Länge: {len(payload)} Bytes")
                        print(f"\n🔬 Debug: Alle 4-Byte-Chunks im Payload (ab Byte 0):")
                        for i in range(0, min(len(payload), 40) - 3, 4):  # Begrenze auf die ersten 40 Bytes
                            chunk = payload[i:i+4]
                            if len(chunk) < 4:
                                break
                            fi = parse_modbus_float_inverse(chunk)
                            be = parse_float32_be(chunk)
                            print(f"  Bytes {i:02}-{i+3:02}: {chunk.hex()} | Floating Inverse: {fi} | Big-Endian: {be}")
                        
                        # Verwende 0x2000 als Standard-Startregister, falls kein letzter Request bekannt ist
                        startreg = 0x2000
                        if 'last_request' in locals() and 'startreg' in last_request:
                            startreg = last_request['startreg']
                            
                        mapped_values = map_values_to_labels(values, startreg)
                        # Wende Plausibilitätsprüfung an
                        mapped_values = apply_plausibility_check(mapped_values)
                        print("📨 Werte (mit Plausibilitätsprüfung):")
                        for k, v in mapped_values.items():
                            unit = UNITS.get(k, "")
                            print(f"  - {k}: {v} {unit}")
                        send_mqtt(mapped_values)
                        # Frame-Länge für Response: 1+1+1+len(payload)+2
                        frame_len = 1+1+1+len(payload)+2
                    # Ungewöhnliche Payload-Länge
                    else:
                        print(f"⚠️  Ungewöhnliche Payload-Länge für Funktionscode {function_code:#04x}: {len(payload)} Bytes bei Adresse {address}")
                        # Versuche, Frame-Länge zu schätzen
                        frame_len = 1+1+len(payload)+2
                else:
                    print(f"⚠️  Funktionscode {function_code:#04x} wird ignoriert (kein Read-Register). Frame wird übersprungen.")
                    # Versuche, Frame-Länge zu schätzen
                    frame_len = 1+1+len(payload)+2

                # Schneide das verarbeitete Frame ab und mache mit dem Rest weiter
                rest_bytes = bytes.fromhex(rest_hex)
                rest_bytes = rest_bytes[frame_len:]
                rest_hex = rest_bytes.hex()

        except Exception as e:
            print(f"❌ Fehler: {e}")

        time.sleep(10)

