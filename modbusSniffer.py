import struct
import paho.mqtt.client as mqtt
import json
import time
import math
import serial  # pyserial, install: pip install pyserial

# DTSU666 Meter Information
# ------------------------
# Der DTSU666 Stromzähler verwendet Modbus RTU mit folgenden Eigenschaften:
# - Adresse des Masters: 0x9F
# - Funktionscodes: 0x03 (Read Holding Registers) und 0x04 (Read Input Registers)
# - Alle Register verwenden das "Floating Inverse (AB CD)" Format (32-Bit-Float)
# - Dies bedeutet, dass die Byte-Reihenfolge umgekehrt wird: von [A,B,C,D] zu [C,D,A,B]

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
    "Uab", "Ubc", "Uca", "Ua", "Ub", "Uc", 
    "Ia", "Ib", "Ic",
    "Pt", "Pa", "Pb", "Pc", 
    "Qt", "Qa", "Qb", "Qc", 
    "PFt", "PFa", "PFb", "PFc", 
    "Freq"
]

# Skalierungsfaktoren für verschiedene Messwerte (basierend auf der DTSU666-Dokumentation)
SCALING_FACTORS = {
    # Spannungen (V): Skalierungsfaktor 0.1 für V
    "Uab": 0.1, "Ubc": 0.1, "Uca": 0.1, "Ua": 0.1, "Ub": 0.1, "Uc": 0.1,
    # Ströme (A): Direktwert ohne Skalierung
    "Ia": 1.0, "Ib": 1.0, "Ic": 1.0,
    # Leistungswerte (W): Direktwert ohne Skalierung
    "Pt": 1.0, "Pa": 1.0, "Pb": 1.0, "Pc": 1.0,
    # Blindleistung (var): Direktwert ohne Skalierung
    "Qt": 1.0, "Qa": 1.0, "Qb": 1.0, "Qc": 1.0,
    # Leistungsfaktoren: Direktwert ohne Skalierung
    "PFt": 1.0, "PFa": 1.0, "PFb": 1.0, "PFc": 1.0,
    # Frequenz (Hz): Direktwert ohne Skalierung
    "Freq": 1.0
}

# Einheiten für die verschiedenen Messwerte
UNITS = {
    "Uab": "V", "Ubc": "V", "Uca": "V", "Ua": "V", "Ub": "V", "Uc": "V",
    "Ia": "A", "Ib": "A", "Ic": "A",
    "Pt": "W", "Pa": "W", "Pb": "W", "Pc": "W",
    "Qt": "var", "Qa": "var", "Qb": "var", "Qc": "var",
    "PFt": "", "PFa": "", "PFb": "", "PFc": "",
    "Freq": "Hz"
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
    Priorisiert die Erkennung von Requests (Adresse 0x9f) und dann korrespondierenden Responses."""
    data = bytes.fromhex(hex_string)
    i = 0
    
    # Priorität 1: Suche nach dem Muster 9F03 (Master-Request für Modbus-Funktion 03/Read Holding Registers)
    while i < len(data) - 8:  # Mindestens 8 Bytes für einen vollständigen Request benötigt
        if data[i] == 0x9f and data[i+1] in (0x03, 0x04):
            # Prüfe, ob genug Bytes für einen vollständigen Request vorhanden sind
            if i + 8 <= len(data):
                startreg = int.from_bytes(data[i+2:i+4], byteorder='big')
                regcount = int.from_bytes(data[i+4:i+6], byteorder='big')
                # Prüfe, ob Register und Count plausibel sind (typisch für DTSU666)
                if 0x2000 <= startreg <= 0x2200 and 1 <= regcount <= 64:
                    print(f"DEBUG: Master-Request gefunden bei Byte {i}: 9F{data[i+1]:02X} Reg={startreg:04X} Count={regcount}")
                    payload = data[i+2:i+6]
                    return data[i], data[i+1], payload
        i += 1
    
    # Setze den Index zurück
    i = 0
    
    # Priorität 2: Suche nach typischen Slave-Response (andere Adresse, meistens 01)
    while i < len(data) - 5:  # Mindestens 5 Bytes für Header + Byte-Count
        if data[i] != 0x9f and data[i+1] in (0x03, 0x04):
            # Prüfe auf typisches Antwortmuster
            if i + 3 < len(data):
                byte_count = data[i+2]
                frame_len = 3 + byte_count + 2  # Adresse + Funktionscode + Byte-Count + Payload + 2 CRC-Bytes
                
                # Zusätzliche Prüfungen für plausible Antwortrahmen
                if byte_count % 4 == 0 and 4 <= byte_count <= 256 and i + frame_len <= len(data):
                    print(f"DEBUG: Slave-Response gefunden bei Byte {i}: {data[i]:02X}{data[i+1]:02X} ByteCount={byte_count}")
                    payload = data[i+3:i+3+byte_count]
                    # Prüfe, ob die Payload-Länge mit dem Byte-Count übereinstimmt
                    if len(payload) == byte_count:
                        return data[i], data[i+1], payload
        i += 1
    
    # Priorität 3: Gezieltes Suchen nach Request-Response-Paaren
    i = 0
    while i < len(data) - 10:  # Mindestens 8 Bytes für Request + ein paar mehr für Response
        if data[i] == 0x9f and data[i+1] in (0x03, 0x04):
            # Ein möglicher Request gefunden
            request_start = i
            try:
                startreg = int.from_bytes(data[i+2:i+4], byteorder='big')
                regcount = int.from_bytes(data[i+4:i+6], byteorder='big')
                
                # Nur fortfahren, wenn die Anforderungsdaten plausibel sind
                if 0x2000 <= startreg <= 0x2200 and 1 <= regcount <= 64:
                    # Suche nach der Response direkt nach dem Request
                    response_start = request_start + 8  # 8 Bytes für einen kompletten Request
                    
                    # Berechne die erwartete Antwortgröße
                    expected_data_length = regcount * 4  # 4 Bytes pro Register (Float)
                    
                    # Prüfe, ob nach dem Request genügend Bytes für eine Antwort vorhanden sind
                    if response_start + 3 + expected_data_length + 2 <= len(data):  # +3 für Header, +2 für CRC
                        # Prüfe, ob die nächsten Bytes einer Antwort ähneln
                        if data[response_start] != 0x9f and data[response_start+1] in (0x03, 0x04):
                            byte_count = data[response_start+2]
                            
                            # Die Byte-Anzahl sollte zur erwarteten Datenlänge passen
                            if byte_count == expected_data_length:
                                print(f"DEBUG: Request-Response-Paar gefunden! Request bei {request_start}, Response bei {response_start}")
                                # Gib zunächst den Request zurück, die Response wird im nächsten Durchlauf verarbeitet
                                payload = data[request_start+2:request_start+6]
                                return data[request_start], data[request_start+1], payload
            except Exception as e:
                # Bei Fehler in der Verarbeitung fortsetzen
                print(f"Fehler bei der Analyse eines möglichen Request-Response-Paars: {e}")
                pass
        i += 1
    
    # Priorität 4: Behandlung von rohen Datenblöcken, die zwischen 9F03-Mustern liegen
    i = 0
    while i < len(data) - 8:
        if data[i] == 0x9f and data[i+1] in (0x03, 0x04):
            # Beginn eines neuen 9F03-Blocks gefunden
            block_start = i
            
            # Suche nach dem nächsten 9F03-Block
            next_block = i + 8  # Mindestens 8 Bytes für einen kompletten Request
            while next_block < len(data) - 8:
                if data[next_block] == 0x9f and data[next_block+1] in (0x03, 0x04):
                    # Nächster 9F03-Block gefunden
                    
                    # Die Daten zwischen dem Ende des ersten Blocks und dem Beginn des nächsten
                    # könnten eine Response sein
                    if next_block - (block_start + 8) >= 8:  # Mindestens 8 Bytes für eine sinnvolle Response
                        # Versuche, die Daten als Response zu interpretieren
                        response_data = data[block_start+8:next_block]
                        print(f"DEBUG: Mögliche Response zwischen 9F03-Blöcken gefunden: Länge={len(response_data)} Bytes")
                        
                        # Wenn der erste Block eine gültige Anfrage ist, gib ihn zurück
                        try:
                            startreg = int.from_bytes(data[block_start+2:block_start+4], byteorder='big')
                            regcount = int.from_bytes(data[block_start+4:block_start+6], byteorder='big')
                            
                            if 0x2000 <= startreg <= 0x2200 and 1 <= regcount <= 64:
                                payload = data[block_start+2:block_start+6]
                                return data[block_start], data[block_start+1], payload
                        except:
                            pass
                    
                    break
                next_block += 1
        i += 1
    
    # Letzter Ausweg: Suche nach Float-Blöcken in rohen Daten
    for expected_length in [88, 176]:  # 22 oder 44 Register * 4 Bytes
        i = 0
        while i <= len(data) - expected_length:
            # Überprüfe, ob kurz vor diesem Block ein 9F03-Muster ist
            if i >= 8 and data[i-8] == 0x9f and data[i-7] in (0x03, 0x04):
                # Es könnte sich um eine Response handeln, die auf einen 9F03-Request folgt
                # Überprüfe, ob es einen plausiblen Float-Block gibt
                valid_values = 0
                for j in range(0, min(expected_length, 40), 4):  # Prüfe nur die ersten 10 Floats
                    chunk = data[i+j:i+j+4]
                    if len(chunk) == 4:
                        try:
                            # CDAB-Format (Floating Inverse)
                            reordered = chunk[2:4] + chunk[0:2]
                            val = struct.unpack(">f", reordered)[0]
                            if not math.isnan(val) and not math.isinf(val) and -1e6 < val < 1e6:
                                valid_values += 1
                        except:
                            pass
                
                # Wenn wir genügend gültige Werte finden, behandle es als Response-Payload
                if valid_values >= 5:  # Mindestens 5 gültige Float-Werte gefunden
                    print(f"DEBUG: Float-Block nach 9F03-Request gefunden: {valid_values} gültige Werte")
                    # Gib den vorangehenden Request zurück
                    req_pos = i - 8
                    payload = data[req_pos+2:req_pos+6]
                    return data[req_pos], data[req_pos+1], payload
            i += 4  # Springe in 4-Byte-Schritten (Float-Alignment)
    
    # Kein gültiger Frame gefunden
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
    
    # Prüfe, ob das Startregister in unserer Registerkarte ist
    register_in_map = start_register in REGISTER_MAP
    
    # Debug-Ausgabe für das Mapping
    print(f"Mapping: Startregister=0x{start_register:04X}, Register in Map: {register_in_map}")
    
    if register_in_map:
        # Wenn das Startregister in der Map ist, mappen wir die Werte auf unsere bekannten Labels
        start_label = REGISTER_MAP[start_register]
        start_index = LABELS.index(start_label) if start_label in LABELS else 0
        
        print(f"  Startlabel: {start_label}, Startindex: {start_index}")
        
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
                register_addr = start_register + i*2
                result[f"Register{register_addr:04X}"] = round(value, 3)
    else:
        # Wenn das Startregister nicht in der Map ist, verwenden wir generische Register-Namen
        print(f"  Unbekanntes Startregister, verwende generische Namen")
        for i, value in enumerate(values):
            register_addr = start_register + i*2
            result[f"Register{register_addr:04X}"] = round(value, 3)
    
    return result

def apply_plausibility_check(values_dict):
    """Prüft, ob die Werte physikalisch plausibel sind und ersetzt unplausible Werte durch 0."""
    plausibility_ranges = {
        # Spannung (V): 0-500V (typisch 230V-400V)
        "Uab": (0, 500), "Ubc": (0, 500), "Uca": (0, 500), 
        "Ua": (0, 500), "Ub": (0, 500), "Uc": (0, 500),
        # Strom (A): 0-100A (basierend auf Messbereich des DTSU666)
        "Ia": (0, 100), "Ib": (0, 100), "Ic": (0, 100),
        # Leistung (W): -50000 bis 50000W
        "Pt": (-50000, 50000), "Pa": (-50000, 50000), "Pb": (-50000, 50000), "Pc": (-50000, 50000),
        # Blindleistung (var): -50000 bis 50000var
        "Qt": (-50000, 50000), "Qa": (-50000, 50000), "Qb": (-50000, 50000), "Qc": (-50000, 50000),
        # Leistungsfaktoren: -1 bis 1
        "PFt": (-1, 1), "PFa": (-1, 1), "PFb": (-1, 1), "PFc": (-1, 1),
        # Frequenz (Hz): 45-65Hz (typisch 50Hz oder 60Hz)
        "Freq": (45, 65)
    }
    
    result = {}
    for key, value in values_dict.items():
        # Prüfe, ob es ein bekanntes Label ist oder ein generischer Registername
        if key.startswith("Register"):
            # Für generische Register führen wir keine Plausibilitätsprüfung durch
            result[key] = value
        else:
            # Bekanntes Label mit definierten Plausibilitätsbereichen
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

def process_modbus_payload(payload: bytes, debug=False, force_format=None):
    """Verarbeitet einen Modbus-Payload und extrahiert Float-Werte im 'Floating Inverse (AB CD)' Format.
    Filtert dabei Modbus-Protokollmarker heraus und unterstützt verschiedene Datenformate.
    
    Args:
        payload: Die zu verarbeitenden Payload-Bytes
        debug: True für erweiterte Debug-Ausgaben
        force_format: Format erzwingen ('float32' für 32-Bit-Floats, 'int16' für 16-Bit-Integer)
    
    Returns:
        Liste der extrahierten Werte
    """
    if len(payload) < 2:  # Mindestens 2 Bytes für ein 16-Bit-Register
        print(f"⚠️ Payload zu kurz: {len(payload)} Bytes")
        return []
        
    # Bereinige die Payload von möglichen eingebetteten Modbus-Protokollmarkern
    cleaned_payload = bytearray()
    i = 0
    
    # Debug: Zeige die ursprünglichen Payload-Bytes im Hex-Format
    if debug:
        print(f"Original Payload (hex): {payload.hex()}")
        print(f"Original Payload-Länge: {len(payload)} Bytes")
    
    # Prüfe auf typische Modbus-Marker wie 9F03/9F04 (Master-Requests) oder XX03/XX04 (Slave-Responses)
    while i < len(payload):
        is_protocol_marker = False
        marker_type = ""
        skip_bytes = 0
        
        # Prüfe auf Master-Requests (9F03/9F04)
        if i + 1 < len(payload) and payload[i] == 0x9F and payload[i+1] in (0x03, 0x04):
            is_protocol_marker = True
            marker_type = "Master-Request"
            # Master-Request-Struktur: Adresse(1) + Funktion(1) + Startregister(2) + Anzahl(2) + CRC(2)
            skip_bytes = min(8, len(payload) - i)
        
        # Prüfe auf Slave-Responses (XX03/XX04 gefolgt von Byte-Count)
        elif i + 2 < len(payload) and payload[i] != 0x9F and payload[i+1] in (0x03, 0x04):
            byte_count = payload[i+2]
            # Erweiterte Prüfung für verschiedene Registertypen
            if byte_count > 0 and (byte_count % 2 == 0) and byte_count <= 250:
                is_protocol_marker = True
                marker_type = "Slave-Response"
                # Slave-Response-Struktur: Adresse(1) + Funktion(1) + ByteCount(1) + Daten(byte_count) + CRC(2)
                skip_bytes = min(3 + byte_count + 2, len(payload) - i)
        
        if is_protocol_marker:
            if debug:
                print(f"⚠️ {marker_type}-Marker bei Position {i} gefunden: {payload[i:i+min(4, len(payload)-i)].hex()}")
                if skip_bytes > 0:
                    print(f"   Überspringe {skip_bytes} Bytes")
            i += skip_bytes
        else:
            # Kein Protokollmarker, füge das Byte zur bereinigten Payload hinzu
            cleaned_payload.append(payload[i])
            i += 1
    
    # Konvertiere die bereinigte Liste zurück in Bytes
    cleaned_payload = bytes(cleaned_payload)
    
    if debug:
        print(f"Bereinigte Payload-Länge: {len(cleaned_payload)} Bytes")
        if len(cleaned_payload) < 100:
            print(f"Bereinigte Payload (hex): {cleaned_payload.hex()}")
    
    values = []
    
    # Bestimme das beste Datenformat, wenn nicht erzwungen
    data_format = force_format
    if not data_format:
        # Für den DTSU666 wissen wir, dass alle Register 32-Bit-Floats im Floating Inverse Format (AB CD) sind
        # Wir sollten daher float32 bevorzugen, sofern es keine klaren Gegenhinweise gibt
        
        # Prüfe, ob die Payload-Länge mit dem Float32-Format kompatibel ist
        if len(cleaned_payload) % 4 == 0:
            # Berechne, wie gut die Daten als 32-Bit-Floats interpretiert werden können
            float_valid_count = 0
            float_plausible_count = 0
            total_chunks = len(cleaned_payload) // 4
            
            for i in range(total_chunks):
                chunk = cleaned_payload[i*4:i*4+4]
                value = parse_modbus_float_inverse(chunk)
                
                # Prüfe auf grundlegende Gültigkeit
                if value is not None and not math.isnan(value) and not math.isinf(value) and -1e6 < value < 1e6:
                    float_valid_count += 1
                    
                    # Detailliertere Plausibilitätsprüfung für typische DTSU666-Werte
                    if (0 <= value <= 500 or  # Spannung (V)
                        0 <= value <= 100 or  # Strom (A)
                        45 <= value <= 65 or  # Frequenz (Hz)
                        -1 <= value <= 1 or   # Leistungsfaktor
                        -50000 <= value <= 50000):  # Leistung (W/var)
                        float_plausible_count += 1
            
            # Berechne Gültigkeits- und Plausibilitätsraten
            float_valid_percentage = float_valid_count / total_chunks if total_chunks > 0 else 0
            float_plausible_percentage = float_plausible_count / total_chunks if total_chunks > 0 else 0
            
            # DTSU666 verwendet immer float32 für Register - setze die Schwelle niedriger an
            # Schon wenn 30% der Werte plausibel sind, sollte es als float32 betrachtet werden
            if float_plausible_percentage >= 0.3 or float_valid_percentage >= 0.5:
                data_format = 'float32'
                if debug:
                    print(f"Auto-Erkennung: 'float32' Format (Gültigkeitsrate: {float_valid_percentage:.2f}, Plausibilitätsrate: {float_plausible_percentage:.2f})")
            else:
                # Nur wenn die Plausibilitätsprüfung eindeutig fehlschlägt, verwenden wir int16
                data_format = 'int16'
                if debug:
                    print(f"Auto-Erkennung: 'int16' Format (float32 Gültigkeitsrate zu niedrig: {float_valid_percentage:.2f}, Plausibilitätsrate: {float_plausible_percentage:.2f})")
        else:
            # Wenn die Länge nicht durch 4 teilbar ist, können wir float32 nicht verwenden
            data_format = 'int16'
            if debug:
                print("Auto-Erkennung: 'int16' Format (Payload-Länge nicht durch 4 teilbar)")
    
    if debug:
        print(f"Verwende Datenformat: {data_format}")
    
    invalid_count = 0
    
    if data_format == 'float32':
        # 32-Bit-Float-Verarbeitung (4 Bytes pro Wert)
        max_values = len(cleaned_payload) // 4
        
        for i in range(max_values):
            offset = i * 4
            chunk = cleaned_payload[offset:offset+4]
            
            if len(chunk) != 4:
                if debug:
                    print(f"⚠️ Unvollständiger Chunk an Position {i}: nur {len(chunk)} von 4 Bytes")
                continue
            
            # Letzte Prüfung, ob der Chunk trotzdem Protokollmarker enthält
            if chunk[0:2] == b'\x9f\x03' or chunk[0:2] == b'\x9f\x04':
                if debug:
                    print(f"⚠️ Überspringe übersehenen Protokoll-Marker an Position {i}: Bytes={chunk.hex()}")
                invalid_count += 1
                values.append(0.0)
                continue
            
            # Laut DTSU666-Dokumentation: Float Inverse Format (CDAB)
            value = parse_modbus_float_inverse(chunk)
            
            if value is None or math.isnan(value) or math.isinf(value):
                invalid_count += 1
                if debug:
                    print(f"⚠️ Ungültiger Wert an Position {i}: Bytes={chunk.hex()}")
                    debug_modbus_float_variants(chunk)
                values.append(0.0)  # Ersetze ungültige Werte durch 0
            else:
                values.append(round(value, 3))
    
    elif data_format == 'int16':
        # 16-Bit-Integer-Verarbeitung (2 Bytes pro Wert)
        max_values = len(cleaned_payload) // 2
        
        for i in range(max_values):
            offset = i * 2
            chunk = cleaned_payload[offset:offset+2]
            
            if len(chunk) != 2:
                if debug:
                    print(f"⚠️ Unvollständiger Chunk an Position {i}: nur {len(chunk)} von 2 Bytes")
                continue
            
            # Parsen als 16-Bit-Integer (Big Endian)
            try:
                value = int.from_bytes(chunk, byteorder='big')
                values.append(value)
                if debug and i < 5:  # Zeige nur die ersten Werte für Debug
                    print(f"INT16 an Position {i}: Bytes={chunk.hex()}, Wert={value}")
            except Exception as e:
                invalid_count += 1
                if debug:
                    print(f"⚠️ Fehler beim Parsen des 16-Bit-Integers an Position {i}: {e}")
                values.append(0)
    
    if invalid_count > 0 and debug:
        print(f"⚠️ {invalid_count} von {len(values)} Werten waren ungültig und wurden durch 0 ersetzt")
            
    return values

def parse_modbus_float_inverse(data: bytes, offset=0):
    """Parst einen 4-Byte Chunk als Floating Inverse Format (AB CD).
    
    Für DTSU666-Meter werden ALLE Register im Floating Inverse Format (AB CD) gespeichert.
    Dies bedeutet eine Umordnung der Bytes von [A, B, C, D] zu [C, D, A, B]
    bzw. Vertauschung der beiden Wort-Hälften."""
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

def validate_float_block(data: bytes, min_valid_percentage=0.3, debug=False):
    """Überprüft, ob ein Byte-Block gültige Float-Werte im 'Floating Inverse (AB CD)' Format enthält.
    
    Args:
        data: Der zu prüfende Byte-Block
        min_valid_percentage: Minimaler Prozentsatz an gültigen Werten, um den Block als gültig zu betrachten
        debug: Debug-Ausgaben aktivieren
    
    Returns:
        True, wenn der Block gültige Float-Werte enthält, sonst False
    """
    if len(data) < 8 or len(data) % 4 != 0:
        if debug:
            print(f"⚠️ Ungültige Blocklänge für Float-Validierung: {len(data)} Bytes")
        return False
    
    valid_count = 0
    plausible_values_count = 0
    total_floats = len(data) // 4
    
    # Prüfe auf Protokollmarker
    contains_markers = False
    for i in range(0, len(data) - 1):
        if (i + 1 < len(data) and data[i] == 0x9F and data[i+1] in (0x03, 0x04)) or \
           (i + 2 < len(data) and data[i] != 0x9F and data[i+1] in (0x03, 0x04) and data[i+2] % 4 == 0):
            contains_markers = True
            if debug:
                print(f"⚠️ Block enthält Protokollmarker an Position {i}: {data[i:i+4].hex() if i+4 <= len(data) else data[i:].hex()}")
    
    # Prüfe Floatwerte
    for i in range(0, len(data), 4):
        chunk = data[i:i+4]
        if len(chunk) != 4:
            continue
        
        # Prüfe auf bekannte Protokollmarker innerhalb des Chunks
        if chunk[0] == 0x9F and chunk[1] in (0x03, 0x04):
            if debug:
                print(f"⚠️ Chunk enthält Protokollmarker: {chunk.hex()}")
            continue
            
        # Laut DTSU666-Dokumentation: Float Inverse Format (AB CD)
        reordered = chunk[2:4] + chunk[0:2]
        try:
            value = struct.unpack(">f", reordered)[0]
            
            # Grundlegende Gültigkeitsprüfung: Nicht NaN, nicht Inf, im plausiblen Bereich
            if not math.isnan(value) and not math.isinf(value) and -1e6 < value < 1e6:
                valid_count += 1
                
                # Detailliertere Plausibilitätsprüfung für typische DTSU666-Werte
                if (0 <= value <= 500 or  # Spannung (V)
                    0 <= value <= 100 or  # Strom (A)
                    45 <= value <= 65 or  # Frequenz (Hz)
                    -1 <= value <= 1 or   # Leistungsfaktor
                    -50000 <= value <= 50000):  # Leistung (W/var)
                    plausible_values_count += 1
        except Exception:
            # Bei Fehler in der Dekodierung: ungültiger Float
            pass
    
    # Berechne die Gültigkeits- und Plausibilitätsraten
    valid_percentage = valid_count / total_floats if total_floats > 0 else 0
    plausible_percentage = plausible_values_count / total_floats if total_floats > 0 else 0
    
    # DTSU666 verwendet immer float32 für Register - daher niedrigere Schwelle
    # Akzeptiere den Block, wenn mehr als min_valid_percentage plausible Werte enthält
    # oder mindestens 40% der Werte grundsätzlich gültig sind
    result = plausible_percentage >= min_valid_percentage or valid_percentage >= 0.4
    
    if debug:
        print(f"Float-Block-Validierung: {valid_count}/{total_floats} gültige Werte ({valid_percentage:.2f})")
        print(f"Float-Block-Plausibilität: {plausible_values_count}/{total_floats} plausible Werte ({plausible_percentage:.2f})")
        print(f"Float-Block-Validierungsergebnis: {'✅ Gültig' if result else '❌ Ungültig'}")
        
        if contains_markers:
            print(f"⚠️ Block enthält mögliche Protokollmarker, aber {plausible_percentage:.2f} plausible Werte")
    
    return result
    
    valid_percentage = valid_count / total_floats if total_floats > 0 else 0
    plausible_percentage = plausible_values_count / total_floats if total_floats > 0 else 0
    
    if debug:
        print(f"Float-Block-Validierung: {valid_count}/{total_floats} gültige Werte ({valid_percentage:.1%})")
        print(f"Davon {plausible_values_count} plausible Werte ({plausible_percentage:.1%})")
        
        if contains_markers:
            print("⚠️ Der Block enthält Modbus-Protokollmarker!")
    
    # Wenn der Block Protokollmarker enthält, senken wir die Anforderung an die gültigen Werte
    if contains_markers:
        # Wir akzeptieren den Block trotzdem, wenn mindestens 30% plausible Werte enthalten sind
        return plausible_percentage >= 0.3
    else:
        # Ohne Protokollmarker verwenden wir den normalen Schwellenwert
        return valid_percentage >= min_valid_percentage

def process_request_response_pair(request_data, response_data, debug=False):
    """Verarbeitet ein Request-Response-Paar von Modbus-Daten.
    
    Args:
        request_data: Bytes des Request-Frames (inkl. Adresse, Funktionscode)
        response_data: Bytes des Response-Frames (inkl. Adresse, Funktionscode)
        debug: Debug-Ausgaben aktivieren
    
    Returns:
        Dictionary mit den verarbeiteten Daten oder None bei Fehlern
    """
    if len(request_data) < 8 or len(response_data) < 5:
        if debug:
            print("⚠️ Request oder Response zu kurz")
        return None
    
    # Extrahiere Request-Informationen
    req_address = request_data[0]
    req_function = request_data[1]
    
    # Prüfe, ob es ein plausibler Modbus-Request ist
    if req_address != 0x9F or req_function not in (0x03, 0x04):
        if debug:
            print(f"⚠️ Ungültiger Request: Adresse={req_address:02X}, Funktion={req_function:02X}")
        return None
    
    # Extrahiere Startregister und Anzahl der Register
    try:
        startreg = int.from_bytes(request_data[2:4], byteorder='big')
        regcount = int.from_bytes(request_data[4:6], byteorder='big')
        
        # Plausibilitätsprüfung der Register
        if not (0x2000 <= startreg <= 0x2200) or not (1 <= regcount <= 64):
            if debug:
                print(f"⚠️ Unplausible Register-Werte: Startregister=0x{startreg:04X}, Anzahl={regcount}")
            return None
    except Exception as e:
        if debug:
            print(f"⚠️ Fehler beim Extrahieren der Register-Informationen: {e}")
        return None
    
    # Extrahiere Response-Informationen
    resp_address = response_data[0]
    resp_function = response_data[1]
    
    # Prüfe, ob es eine plausible Modbus-Response ist
    if resp_address == 0x9F or resp_function not in (0x03, 0x04):
        if debug:
            print(f"⚠️ Ungültige Response: Adresse={resp_address:02X}, Funktion={resp_function:02X}")
        return None
    
    # Extrahiere Byte-Count und Payload
    try:
        byte_count = response_data[2]
        
        # Plausibilitätsprüfung des Byte-Counts
        expected_byte_count = regcount * 4  # 4 Bytes pro Float-Register
        
        if byte_count != expected_byte_count:
            if debug:
                print(f"⚠️ Byte-Count passt nicht zur Registeranzahl: Byte-Count={byte_count}, erwartet={expected_byte_count}")
            # Wenn der Byte-Count nicht exakt passt, aber nah genug ist, versuchen wir trotzdem die Daten zu extrahieren
            if abs(byte_count - expected_byte_count) > 8:  # Mehr als 2 Register Unterschied
                if debug:
                    print("⚠️ Zu große Abweichung im Byte-Count, abgebrochen")
                return None
        
        # Extrahiere die eigentlichen Daten
        payload = response_data[3:3+byte_count]
        
        # Prüfe auf eingebettete Protokoll-Marker in der Payload
        for i in range(0, len(payload) - 1):
            if (payload[i] == 0x9F and payload[i+1] in (0x03, 0x04)) or \
               (payload[i] != 0x9F and payload[i+1] in (0x03, 0x04) and i+2 < len(payload) and payload[i+2] % 4 == 0):
                if debug:
                    print(f"⚠️ Eingebetteter Protokoll-Marker in der Payload bei Position {i}: {payload[i:i+4].hex() if i+4 <= len(payload) else payload[i:].hex()}")
                
                # Bereinige die Payload durch Verwendung der verbesserten Prozessfunktion
                if debug:
                    print("🔄 Bereinige Payload von Protokoll-Markern...")
        
        # Validiere den Float-Block
        if not validate_float_block(payload, min_valid_percentage=0.3, debug=debug):
            if debug:
                print("⚠️ Float-Block-Validierung fehlgeschlagen, aber Verarbeitung wird fortgesetzt")
            # Wir versuchen trotzdem die Daten zu verarbeiten
    except Exception as e:
        if debug:
            print(f"⚠️ Fehler beim Extrahieren der Response-Daten: {e}")
        return None
    
    # Verarbeite die Payload als Float-Werte mit der verbesserten Funktion
    values = process_modbus_payload(payload, debug=debug)
    
    if not values:
        if debug:
            print("⚠️ Keine gültigen Werte in der Payload gefunden")
        return None
    
    # Mappe Werte auf Labels und wende Plausibilitätsprüfung an
    mapped_values = map_values_to_labels(values, startreg)
    mapped_values = apply_plausibility_check(mapped_values)
    
    return {
        "request": {
            "address": req_address,
            "function": req_function,
            "startreg": startreg,
            "regcount": regcount
        },
        "response": {
            "address": resp_address,
            "function": resp_function,
            "byte_count": byte_count,
            "values": values
        },
        "mapped_values": mapped_values
    }

def extract_and_process_response(data_buffer, position, function_code, last_request=None, debug=True):
    """Extrahiert und verarbeitet eine Response an der angegebenen Position im Puffer.
    
    Args:
        data_buffer: Der Puffer mit den Rohdaten
        position: Die Position im Puffer, an der die Response beginnt
        function_code: Der erwartete Funktionscode
        last_request: Optional, der letzte bekannte Request für die Korrelation
        debug: Debug-Ausgaben aktivieren
    
    Returns:
        Tuple aus (Erfolg, Response-Länge, Gemappte Werte)
    """
    if position + 3 >= len(data_buffer):
        if debug:
            print(f"⚠️ Nicht genug Bytes für eine Response an Position {position}")
        return False, 0, None
    
    # Extrahiere Adresse, Funktionscode und Byte-Count
    address = data_buffer[position]
    func_code = data_buffer[position + 1]
    byte_count = data_buffer[position + 2]
    
    # Zeige Bytes für besseres Debugging
    if debug:
        debug_bytes = data_buffer[position:position+min(20, len(data_buffer)-position)]
        print(f"DEBUG: Bytes an Position {position}: {debug_bytes.hex()}")
    
    # Prüfe auf plausible Werte
    if address == 0x9F:
        if debug:
            print(f"⚠️ Keine Response: Adresse ist 0x9F (Master) an Position {position}")
        return False, 0, None
    
    # Lockerere Funktionscode-Prüfung - auch Varianten zulassen (z.B. 0x83 als Fehlercode für 0x03)
    if func_code != function_code and func_code != (function_code | 0x80):
        if debug:
            print(f"⚠️ Funktionscode {func_code:02X} stimmt nicht mit erwartetem Code {function_code:02X} überein")
        return False, 0, None
    
    # Prüfe, ob es ein Fehlercode ist (Bit 7 gesetzt)
    if func_code & 0x80:
        if debug:
            print(f"⚠️ Modbus-Fehlercode empfangen: {func_code:02X}")
            if position + 3 < len(data_buffer):
                error_code = data_buffer[position + 2]
                print(f"  Fehlercode: {error_code:02X}")
        # Trotzdem als Erfolg behandeln, aber keine Werte zurückgeben
        return True, 5, None  # Adresse(1) + Funktionscode(1) + Fehlercode(1) + CRC(2)
    
    # Verbesserte Byte-Count-Prüfung
    # Berechne die erwartete Byte-Count-Größe basierend auf der letzten Anfrage
    expected_byte_count = 0
    if last_request:
        regcount = last_request.get('regcount', 0)
        if regcount > 0:
            # Für DTSU666 erwarten wir immer 4 Bytes pro Register (32-Bit-Float)
            expected_byte_count = regcount * 4  # 4 Bytes pro Register für Float32
            if debug:
                print(f"ℹ️ Erwarteter Byte-Count für {regcount} Register: {expected_byte_count} Bytes")
                print(f"ℹ️ Tatsächlicher Byte-Count in der Response: {byte_count} Bytes")
                
            # Wenn Byte-Count und erwarteter Wert nicht übereinstimmen, ist das eine Warnung
            if byte_count != expected_byte_count:
                if debug:
                    print(f"⚠️ Abweichender Byte-Count: {byte_count} statt {expected_byte_count}")
    
    # Erweiterte Plausibilitätsprüfung für den Byte-Count
    if byte_count == 0 or byte_count > 250:
        if debug:
            print(f"⚠️ Unplausibler Byte-Count: {byte_count} an Position {position}")
        return False, 0, None
    
    # Für DTSU666: Prüfe, ob der Byte-Count ein Vielfaches von 4 ist (für 32-Bit-Floats)
    if byte_count % 4 != 0 and byte_count % 2 == 0:
        if debug:
            print(f"⚠️ Byte-Count {byte_count} ist kein Vielfaches von 4, möglicherweise INT16-Daten statt FLOAT32")
    elif byte_count % 2 != 0:
        if debug:
            print(f"⚠️ Untypischer Byte-Count (nicht durch 2 teilbar): {byte_count}")
            # Wir akzeptieren es trotzdem, aber geben eine Warnung aus
    
    # Prüfe, ob genug Bytes für die komplette Response vorhanden sind
    response_length = 3 + byte_count + 2  # Adresse(1) + Funktionscode(1) + ByteCount(1) + Daten(byte_count) + CRC(2)
    if position + response_length > len(data_buffer):
        if debug:
            print(f"⚠️ Nicht genug Bytes für eine komplette Response: benötige {response_length}, verfügbar {len(data_buffer) - position}")
        return False, 0, None
    
    # Extrahiere die Response-Daten
    response_data = data_buffer[position:position + response_length]
    
    if debug:
        print(f"✓ Response extrahiert: Adresse={address:02X}, Funktion={func_code:02X}, ByteCount={byte_count}")
        print(f"  Response-Daten: {response_data[:10].hex()}{'...' if len(response_data) > 10 else ''}")
    
    # Extrahiere die Payload (Daten nach dem Byte-Count)
    payload = response_data[3:3 + byte_count]
    
    # Prüfe CRC, falls vorhanden
    if len(response_data) >= 5:  # Mindestens Adresse + Funktion + ByteCount + 1 Datenbyte + 2 CRC-Bytes
        # Hier könnte eine CRC-Prüfung hinzugefügt werden, falls nötig
        pass
    
    # Verarbeite die Payload - für DTSU666 immer als Float32 interpretieren, wenn möglich
    force_format = None
    if byte_count % 4 == 0:
        # Für DTSU666 verwenden wir bevorzugt das Float32-Format
        force_format = 'float32'
        if debug:
            print("  Verwende bevorzugt FLOAT32-Format (typisch für DTSU666)")
    
    values = process_modbus_payload(payload, debug=debug, force_format=force_format)
    
    if not values:
        if debug:
            print("⚠️ Keine gültigen Werte in der Payload gefunden")
            # Versuche direkte Interpretation als 16-Bit-Register nur als Fallback
            if byte_count % 2 == 0:
                print("  Versuche alternative Interpretation als 16-Bit-Register...")
                int_values = []
                for i in range(0, len(payload), 2):
                    if i + 2 <= len(payload):
                        val = int.from_bytes(payload[i:i+2], byteorder='big')
                        int_values.append(val)
                if int_values:
                    print(f"  16-Bit-Interpretation: {int_values}")
                    # Versuche trotzdem zu mappen, wenn INT16-Werte gefunden wurden
                    startreg = 0x2000
                    if last_request:
                        startreg = last_request.get('startreg', 0x2000)
                    mapped_values = map_values_to_labels(int_values, startreg)
                    mapped_values = apply_plausibility_check(mapped_values)
                    return True, response_length, mapped_values
        return True, response_length, None
    
    # Bestimme das Startregister aus dem letzten Request oder verwende den Standardwert
    startreg = 0x2000
    if last_request:
        startreg = last_request.get('startreg', 0x2000)
    
    if debug:
        print(f"  Verwende Startregister 0x{startreg:04X} für die Zuordnung der Werte")
    
    # Mappe Werte auf Labels und wende Plausibilitätsprüfung an
    mapped_values = map_values_to_labels(values, startreg)
    mapped_values = apply_plausibility_check(mapped_values)
    
    return True, response_length, mapped_values

def scan_buffer_for_responses(data_buffer, requests_found, last_request=None, debug=True):
    """Führt eine fortschrittliche Analyse des Puffers durch, um Slave-Antworten zu erkennen.
    
    Args:
        data_buffer: Der zu analysierende Puffer
        requests_found: Liste der bereits erkannten Requests
        last_request: Der letzte bekannte Request
        debug: Debug-Ausgaben aktivieren
    
    Returns:
        Tuple aus (erfolg, position, länge, gemappte_werte)
    """
    if not requests_found and not last_request:
        if debug:
            print("⚠️ Keine Requests zum Korrelieren gefunden")
        return False, 0, 0, None
    
    # Die Antworten könnten in einem anderen Format sein als erwartet
    # Typische Slave-Geräte-IDs für den DTSU666 können variieren
    # Wir versuchen eine breitere Auswahl an möglichen IDs
    all_slave_ids = list(range(1, 64))
    # Prioritätsliste für die häufigsten IDs zuerst
    priority_slave_ids = [1, 2, 3, 10, 11, 12, 20, 30]
    
    # Sortiere die Liste, um zuerst die Prioritäts-IDs zu prüfen
    slave_ids = priority_slave_ids + [id for id in all_slave_ids if id not in priority_slave_ids and id != 0x9F]
    
    if debug:
        print("🔍 Diagnosescan des gesamten Puffers...")
        
        # Scanne den Puffer nach bestimmten Byte-Mustern
        patterns_found = {}
        for i in range(0, min(len(data_buffer) - 3, 1000)):  # Begrenzen auf die ersten 1000 Bytes
            # Speichere jedes 4-Byte-Muster und zähle Vorkommen
            pattern = data_buffer[i:i+4].hex()
            patterns_found[pattern] = patterns_found.get(pattern, 0) + 1
        
        # Zeige die häufigsten Muster
        print("Häufigste Muster im Puffer:")
        common_patterns = sorted(patterns_found.items(), key=lambda x: x[1], reverse=True)[:5]
        for pattern, count in common_patterns:
            print(f"  Muster: {pattern}, Häufigkeit: {count}")
    
    # Methode 1: Standardsuche nach Slave-Responses
    if debug:
        print("\n1️⃣ Suche nach Standard-Modbus-Antworten...")
    
    for slave_id in slave_ids:
        i = 0
        while i < len(data_buffer) - 5:
            # Suche nach Slave-ID gefolgt von Funktionscode 0x03/0x04
            if data_buffer[i] == slave_id and data_buffer[i+1] in (0x03, 0x04):
                if debug:
                    print(f"DEBUG: Potenzielle Response mit Slave-ID {slave_id} bei Position {i}")
                # Versuche, eine Response zu extrahieren und zu verarbeiten
                success, resp_length, mapped_values = extract_and_process_response(
                    data_buffer, i, data_buffer[i+1], last_request, debug
                )
                
                if success and mapped_values:
                    if debug:
                        print(f"\n✅ Response von Slave-ID {slave_id} erfolgreich verarbeitet!")
                    return True, i, resp_length, mapped_values
            i += 1
    
    # Methode 2: Suche nach Antworten direkt nach den Requests
    if debug:
        print("\n2️⃣ Suche nach Antworten direkt nach den Requests...")
    
    for req in requests_found:
        req_pos = req['position']
        req_end = req_pos + 8  # 8 Bytes für einen vollständigen Request
        
        # Berechne die erwartete Antwortgröße basierend auf der Registeranzahl
        regcount = req['regcount']
        expected_data_length_32bit = regcount * 4  # 32-Bit-Float (4 Bytes pro Register)
        
        # Prüfe, ob genügend Bytes nach dem Request vorhanden sind
        if req_end + 5 + expected_data_length_32bit <= len(data_buffer):
            # Zeige die Bytes nach dem Request
            after_req = data_buffer[req_end:req_end+10]
            if debug:
                print(f"Bytes nach Request bei Position {req_pos}: {after_req.hex()}")
            
            # Versuche, die verschiedenen möglichen Antwortformate zu erkennen
            for offset in range(0, 20):  # Prüfe verschiedene Offsets nach dem Request
                pos = req_end + offset
                if pos + 5 >= len(data_buffer):
                    break
                
                # Prüfe auf verschiedene Funktionscodes (0x03, 0x04)
                for func_code in (0x03, 0x04):
                    # Prüfe, ob die Bytes an dieser Position einer Antwort ähneln
                    success, resp_length, mapped_values = extract_and_process_response(
                        data_buffer, pos, func_code, last_request=req, debug=False
                    )
                    
                    if success and mapped_values:
                        if debug:
                            print(f"\n✅ Response nach Request gefunden bei Offset +{offset}!")
                        return True, pos, resp_length, mapped_values
    
    # Methode 3: Rohe Datenblockinterpretation
    if debug:
        print("\n3️⃣ Versuche direkte Datenblockinterpretation...")
    
    # Verwende den neuesten Request, falls vorhanden
    latest_req = requests_found[-1] if requests_found else last_request
    
    if latest_req:
        # Extrahiere Registerinformationen
        if isinstance(latest_req, dict):
            req_pos = latest_req.get('position', 0)
            startreg = latest_req.get('startreg', 0x2000)
            regcount = latest_req.get('regcount', 44)  # Standardwert 44 Register
        else:
            req_pos = 0
            startreg = latest_req.get('startreg', 0x2000)
            regcount = latest_req.get('regcount', 44)
        
        # Suche nach einem Datenblock nach dem Request
        search_start = req_pos + 8 if req_pos > 0 else 0
        search_end = min(search_start + 300, len(data_buffer) - regcount*4)
        
        # Versuche, jeden möglichen Startpunkt als Anfang eines Datenblocks zu interpretieren
        for block_start in range(search_start, search_end, 4):  # In 4-Byte-Schritten für Float-Alignment
            # Versuche, einen Block von 4*regcount Bytes als Float-Werte zu interpretieren
            if block_start + regcount*4 <= len(data_buffer):
                data_block = data_buffer[block_start:block_start+regcount*4]
                
                # Prüfe, ob es gültige Float-Werte enthält
                if debug and block_start % 20 == 0:  # Nur jeden 20. Block für weniger Ausgabe
                    print(f"Prüfe Float32-Datenblock bei Position {block_start}...")
                
                # Verwende die vorhandene Funktion zum Validieren von Float-Blöcken
                # Für DTSU666: Niedrigere Schwelle von 0.3 für Plausibilitätsvalidierung
                if validate_float_block(data_block, min_valid_percentage=0.3, debug=False):
                    if debug:
                        print(f"✅ Potenziell gültiger Float-Block gefunden bei Position {block_start}!")
                        print(f"  Block-Daten (erste 16 Bytes): {data_block[:16].hex()}")
                    
                    # Verwende die vorhandene Funktion zum Parsen von Float-Werten
                    # DTSU666 verwendet immer float32-Format - erzwinge es
                    values = process_modbus_payload(data_block, debug=debug, force_format='float32')
                    
                    if values and len(values) >= regcount * 0.7:  # Mindestens 70% der erwarteten Werte
                        if debug:
                            print(f"✅ Gültiger Datenblock mit {len(values)} Werten gefunden!")
                        
                        # Mappe die Werte auf Register
                        mapped_values = map_values_to_labels(values, startreg)
                        mapped_values = apply_plausibility_check(mapped_values)
                        
                        # Prüfe, ob die gemappten Werte sinnvoll sind
                        if len(mapped_values) > 0 and any(not k.startswith("Register") for k in mapped_values.keys()):
                            return True, block_start, regcount*4, mapped_values
    
    return False, 0, 0, None
if __name__ == "__main__":
    # Konfiguration
    serial_port = "/dev/ttyUSB0"  # passe ggf. an
    baudrate = 9600               # passe ggf. an
    
    # Puffer-Konfiguration
    MAX_BUFFER_SIZE = 4096  # 4 KB sollte für lange Datenblöcke ausreichend sein
    data_buffer = bytearray()
    
    # Speichere den letzten Request für die Korrelation mit nachfolgenden Responses
    last_request = None
    
    print("🔄 Modbus RTU Sniffer für DTSU666 gestartet")
    print(f"📊 Serielle Schnittstelle: {serial_port} mit {baudrate} Baud")
    
    while True:
        try:
            print("\n📡 Warte auf Daten vom Zähler...")
            
            # Lese Daten von der seriellen Schnittstelle
            serial_data = read_from_serial(serial_port, baudrate)
            if not serial_data:
                time.sleep(1)  # Kurze Pause bei leeren Daten
                continue

            # Konvertiere den Hex-String in Bytes und füge ihn zum Buffer hinzu
            data_bytes = bytes.fromhex(serial_data)
            data_buffer.extend(data_bytes)
            
            # Begrenze die Puffergröße, um Speicherprobleme zu vermeiden
            if len(data_buffer) > MAX_BUFFER_SIZE:
                print(f"⚠️ Puffer-Überlauf! Puffer wird auf {MAX_BUFFER_SIZE} Bytes begrenzt.")
                # Behalte nur die neuesten Daten
                data_buffer = data_buffer[-MAX_BUFFER_SIZE:]
            
            # Zeige Debug-Informationen
            print(f"[RAW] Neue Daten: {serial_data[:60]}{'...' if len(serial_data) > 60 else ''} (Länge: {len(serial_data)//2} Bytes)")
            print(f"[BUFFER] Aktueller Puffer: {len(data_buffer)} Bytes")
            
            # Verarbeitungsstatistik
            frames_processed = 0
            requests_found = []
            responses_found = []
            
            # Spezielle Vorverarbeitung: Entferne bekannte Muster, die keine gültigen Modbus-Frames sind
            i = 0
            while i < len(data_buffer) - 2:
                # Suche nach ungültigen Byte-Sequenzen und entferne sie
                # Beispiel: 0x00 0x00 0x00 Sequenzen (Nullbytes)
                if (i + 3 <= len(data_buffer) and 
                    data_buffer[i] == 0x00 and 
                    data_buffer[i+1] == 0x00 and 
                    data_buffer[i+2] == 0x00):
                    # Drei Nullbytes hintereinander - wahrscheinlich kein gültiger Modbus-Frame
                    del data_buffer[i:i+3]
                    continue
                
                i += 1
            
            # Suche nach Modbus-Requests (0x9F + 0x03/0x04) im Puffer
            i = 0
            while i < len(data_buffer) - 8:  # Mindestens 8 Bytes für einen vollständigen Request
                if data_buffer[i] == 0x9F and data_buffer[i+1] in (0x03, 0x04):
                    # Potenzieller Master-Request gefunden
                    try:
                        # Prüfe, ob genug Bytes für einen vollständigen Request vorhanden sind
                        if i + 8 <= len(data_buffer):
                            startreg = int.from_bytes(data_buffer[i+2:i+4], byteorder='big')
                            regcount = int.from_bytes(data_buffer[i+4:i+6], byteorder='big')
                            
                            # Prüfe, ob Register und Count plausibel sind (typisch für DTSU666)
                            if 0x2000 <= startreg <= 0x2200 and 1 <= regcount <= 64:
                                # Gültiger Request gefunden
                                request_data = data_buffer[i:i+8]  # 8 Bytes für einen kompletten Request
                                requests_found.append({
                                    'position': i,
                                    'data': request_data,
                                    'startreg': startreg,
                                    'regcount': regcount,
                                    'function_code': data_buffer[i+1]
                                })
                                print(f"✓ Master-Request gefunden bei Position {i}: Startregister=0x{startreg:04X}, Anzahl={regcount}")
                    except Exception as e:
                        print(f"⚠️ Fehler beim Parsen eines möglichen Master-Requests bei Position {i}: {e}")
                
                # Suche auch nach Slave-Responses
                elif data_buffer[i] != 0x9F and data_buffer[i+1] in (0x03, 0x04, 0x83, 0x84) and i+2 < len(data_buffer):
                    # Potenzieller Slave-Response gefunden
                    try:
                        # Check for error response
                        if data_buffer[i+1] & 0x80:
                            # Modbus error response
                            if i + 5 <= len(data_buffer):  # Adresse + Funktion + Fehlercode + 2 CRC
                                error_code = data_buffer[i+2]
                                response_data = data_buffer[i:i+5]
                                responses_found.append({
                                    'position': i,
                                    'data': response_data,
                                    'error_code': error_code,
                                    'function_code': data_buffer[i+1] & 0x7F,  # Original function code
                                    'is_error': True
                                })
                                print(f"✓ Slave-Fehler-Response gefunden bei Position {i}: Device={data_buffer[i]:02X}, Fehlercode={error_code:02X}")
                        else:
                            # Regular response
                            byte_count = data_buffer[i+2]
                            
                            # Erweiterte Prüfungen für verschiedene Datentypen
                            is_valid = False
                            
                            # Prüfe auf 16-Bit-Register (2 Bytes pro Register)
                            if byte_count % 2 == 0 and byte_count > 0 and byte_count <= 250:
                                is_valid = True
                            
                            # Prüfe auf 32-Bit-Float-Register (4 Bytes pro Register)
                            if byte_count % 4 == 0 and byte_count > 0 and byte_count <= 250:
                                is_valid = True
                            
                            if is_valid and i + 3 + byte_count + 2 <= len(data_buffer):  # Header + Data + CRC
                                # Gültige Response gefunden
                                response_data = data_buffer[i:i+3+byte_count+2]
                                responses_found.append({
                                    'position': i,
                                    'data': response_data,
                                    'byte_count': byte_count,
                                    'function_code': data_buffer[i+1],
                                    'is_error': False
                                })
                                print(f"✓ Slave-Response gefunden bei Position {i}: Device={data_buffer[i]:02X}, ByteCount={byte_count}, HEX={response_data[:10].hex() if len(response_data) > 10 else response_data.hex()}")
                                
                                # Versuche zu bestimmen, ob es sich um 16-Bit oder 32-Bit Daten handelt
                                if byte_count % 4 == 0:
                                    print(f"  Vermutlich 32-Bit Floats ({byte_count // 4} Register)")
                                elif byte_count % 2 == 0:
                                    print(f"  Vermutlich 16-Bit Register ({byte_count // 2} Register)")
                    except Exception as e:
                        print(f"⚠️ Fehler beim Parsen einer möglichen Slave-Response bei Position {i}: {e}")
                        import traceback
                        traceback.print_exc()
                
                i += 1
            
            # Verarbeite gefundene Requests und suche nach dazugehörigen Responses
            for req_idx, req in enumerate(requests_found):
                # Bestimme den erwarteten Beginn der Response
                resp_start = req['position'] + 8  # Nach dem 8-Byte-Request
                
                # Berechne die erwartete Payload-Länge
                expected_data_length = req['regcount'] * 4  # 4 Bytes pro Register (Float)
                expected_resp_length = 3 + expected_data_length + 2  # Adresse(1) + Funktionscode(1) + ByteCount(1) + Daten + CRC(2)
                
                # Prüfe, ob nach dem Request genügend Bytes für eine Antwort vorhanden sind
                if resp_start + expected_resp_length <= len(data_buffer):
                    # Debug-Ausgabe zur Analyse der Bytes nach dem Request
                    next_bytes = data_buffer[resp_start:resp_start+4].hex()
                    print(f"DEBUG: Bytes nach Request an Position {resp_start}: {next_bytes}")
                    
                    # Prüfe direkt nach dem Request auf eine passende Response
                    if (data_buffer[resp_start] != 0x9F and 
                        (data_buffer[resp_start+1] == req['function_code'] or 
                         data_buffer[resp_start+1] == (req['function_code'] | 0x80))):
                        
                        # Spezialfall: Fehlerantwort
                        if data_buffer[resp_start+1] & 0x80:
                            response_length = 5  # Adresse(1) + Funktion(1) + Fehlercode(1) + CRC(2)
                            if resp_start + response_length <= len(data_buffer):
                                error_code = data_buffer[resp_start+2]
                                response_data = data_buffer[resp_start:resp_start+response_length]
                                print(f"⚠️ Modbus-Fehlercode: {error_code:02X} für Request {req_idx+1}")
                                
                                # Entferne die verarbeitete Response aus dem Puffer
                                if resp_start + response_length <= len(data_buffer):
                                    del data_buffer[:resp_start + response_length]
                                else:
                                    del data_buffer[:resp_start]
                                
                                frames_processed += 1
                                break
                        
                        # Normale Antwort mit Byte-Count prüfen
                        byte_count = data_buffer[resp_start+2]
                        expected_response_length = 3 + byte_count + 2  # Adresse(1) + Funktionscode(1) + ByteCount(1) + Daten(byte_count) + CRC(2)
                        
                        # Versuche verschiedene Formatierungen (16-Bit oder 32-Bit)
                        if byte_count == expected_data_length:
                            # Perfekt passende Antwort für 32-Bit-Float-Werte
                            force_format = 'float32'
                        elif byte_count == expected_data_length // 2:
                            # Passende Antwort für 16-Bit-Register
                            force_format = 'int16'
                        else:
                            # Unbekanntes Format, Auto-Erkennung verwenden
                            force_format = None
                            
                            # Trotzdem Plausibilitätsprüfung
                            if byte_count == 0 or byte_count > 250 or byte_count % 2 != 0:
                                print(f"⚠️ Unplausibler Byte-Count: {byte_count}, erwartet ca. {expected_data_length}")
                                continue
                        
                        # Prüfe, ob genug Bytes für die komplette Response vorhanden sind
                        if resp_start + expected_response_length <= len(data_buffer):
                            response_data = data_buffer[resp_start:resp_start+expected_response_length]
                            print(f"✓ Passende Response direkt nach Request {req_idx+1} gefunden!")
                            print(f"  Response: Adresse={data_buffer[resp_start]:02X}, Funktion={data_buffer[resp_start+1]:02X}, ByteCount={byte_count}, Format={force_format or 'auto'}")
                            
                            # Verarbeite das Request-Response-Paar
                            try:
                                # Extrahiere die Payload
                                payload = response_data[3:3+byte_count]
                                
                                # Verarbeite die Payload mit dem erkannten Format
                                values = process_modbus_payload(payload, debug=True, force_format=force_format)
                                
                                if values:
                                    # Mappe Werte auf Labels basierend auf dem Startregister
                                    mapped_values = map_values_to_labels(values, req['startreg'])
                                    
                                    # Zeige die gemappten Werte an
                                    print("\n📨 Werte (mit Plausibilitätsprüfung):")
                                    for k, v in mapped_values.items():
                                        if k.startswith("Register"):
                                            # Generisches Register ohne Einheit
                                            print(f"  - {k}: {v}")
                                        else:
                                            # Bekanntes Label mit Einheit
                                            unit = UNITS.get(k, "")
                                            print(f"  - {k}: {v} {unit}")
                                    
                                    # Sende Werte per MQTT
                                    send_mqtt(mapped_values)
                                    
                                    frames_processed += 1
                                    
                                    # Entferne die verarbeitete Response aus dem Puffer
                                    if resp_start + expected_response_length <= len(data_buffer):
                                        del data_buffer[:resp_start + expected_response_length]
                                    else:
                                        del data_buffer[:resp_start]
                                    
                                    break  # Beende die Schleife nach erfolgreicher Verarbeitung
                                else:
                                    print("⚠️ Keine gültigen Werte in der Payload gefunden")
                            except Exception as e:
                                print(f"⚠️ Fehler bei der Verarbeitung des Request-Response-Paars: {e}")
                                import traceback
                                traceback.print_exc()
                            
                            # Sende Werte per MQTT
                            send_mqtt(mapped_values)
                            
                            # Aktualisiere den letzten Request
                            last_request = {
                                'address': 0x9F,
                                'function_code': req['function_code'],
                                'startreg': req['startreg'],
                                'regcount': req['regcount'],
                                'timestamp': time.time()
                            }
                            
                            frames_processed += 2  # Request + Response
                            
                            # Entferne die verarbeiteten Frames aus dem Puffer
                            del data_buffer[:resp_start+expected_resp_length]
                            break  # Verarbeite zunächst nur das erste erfolgreiche Paar
                    else:
                        # Keine direkte Response gefunden, suche in einem erweiterten Bereich
                        print(f"DEBUG: Keine direkte Response gefunden, starte erweiterte Suche...")
                        extended_search_end = min(resp_start + 100, len(data_buffer) - expected_resp_length)
                        response_found = False
                        
                        # Prüfe alle Slave-Geräteadressen (typischerweise 0x01 bis 0x3F)
                        slave_addresses = [addr for addr in range(1, 64) if addr != 0x9F]
                        
                        for search_pos in range(resp_start, extended_search_end):
                            if (data_buffer[search_pos] in slave_addresses and 
                                data_buffer[search_pos+1] == req['function_code']):
                                
                                byte_count = data_buffer[search_pos+2]
                                
                                # Zeige Debug-Info zu potenziellen Responses
                                print(f"DEBUG: Potenzielle Response bei {search_pos}: Adresse={data_buffer[search_pos]:02X}, " + 
                                      f"Funktion={data_buffer[search_pos+1]:02X}, ByteCount={byte_count}")
                                
                                # Prüfe, ob die Byte-Anzahl plausibel ist
                                if (byte_count % 4 == 0 and byte_count > 0 and byte_count <= 200 and
                                    search_pos + 3 + byte_count + 2 <= len(data_buffer)):
                                    
                                    # Auch abweichende Längen akzeptieren, wenn sie plausibel sind
                                    if byte_count != expected_data_length:
                                        print(f"  ⚠️ Abweichende Byte-Anzahl: {byte_count} statt {expected_data_length}")
                                        if abs(byte_count - expected_data_length) > 40:  # Mehr als 10 Register Unterschied
                                            print(f"  ⚠️ Zu große Abweichung, ignoriere diesen Frame")
                                            continue
                                    response_data = data_buffer[search_pos:search_pos+3+byte_count+2]
                                    print(f"✓ Passende Response für Request {req_idx+1} bei Position {search_pos} gefunden!")
                                    
                                    # Verarbeite das Request-Response-Paar
                                    result = process_request_response_pair(req['data'], response_data, debug=True)
                                    if result:
                                        # Zeige die gemappten Werte an
                                        mapped_values = result['mapped_values']
                                        print("\n📨 Werte (mit Plausibilitätsprüfung):")
                                        for k, v in mapped_values.items():
                                            if k.startswith("Register"):
                                                # Generisches Register ohne Einheit
                                                print(f"  - {k}: {v}")
                                            else:
                                                # Bekanntes Label mit Einheit
                                                unit = UNITS.get(k, "")
                                                print(f"  - {k}: {v} {unit}")
                                        
                                        # Sende Werte per MQTT
                                        send_mqtt(mapped_values)
                                        
                                        # Aktualisiere den letzten Request
                                        last_request = {
                                            'address': 0x9F,
                                            'function_code': req['function_code'],
                                            'startreg': req['startreg'],
                                            'regcount': req['regcount'],
                                            'timestamp': time.time()
                                        }
                                        
                                        frames_processed += 2  # Request + Response
                                        
                                        # Entferne die verarbeiteten Frames aus dem Puffer
                                        del data_buffer[:search_pos+3+byte_count+2]
                                        response_found = True
                                        break
            
            # Wenn keine Request-Response-Paare gefunden wurden, versuche einzelne Frames zu extrahieren
            if frames_processed == 0:
                # Zunächst versuchen wir, die gefundenen Slave-Responses direkt zu verarbeiten
                for resp_idx, resp in enumerate(responses_found):
                    print(f"\n🔍 Verarbeite gefundene Slave-Response {resp_idx+1}/{len(responses_found)}")
                    success, resp_length, mapped_values = extract_and_process_response(
                        data_buffer, 
                        resp['position'], 
                        resp['function_code'], 
                        last_request
                    )
                    
                    if success and mapped_values:
                        print("\n📨 Werte (mit Plausibilitätsprüfung):")
                        for k, v in mapped_values.items():
                            if k.startswith("Register"):
                                # Generisches Register ohne Einheit
                                print(f"  - {k}: {v}")
                            else:
                                # Bekanntes Label mit Einheit
                                unit = UNITS.get(k, "")
                                print(f"  - {k}: {v} {unit}")
                        
                        # Sende Werte per MQTT
                        send_mqtt(mapped_values)
                        
                        frames_processed += 1
                        
                        # Entferne die verarbeitete Response aus dem Puffer
                        if resp['position'] + resp_length <= len(data_buffer):
                            del data_buffer[:resp['position'] + resp_length]
                            break
                
            # Wenn noch immer keine Frames verarbeitet wurden, suche nach dem ersten gültigen Frame im Hex-Format
            if frames_processed == 0:
                # Konvertiere den Puffer zu einem Hex-String für die bestehende Extraktionsmethode
                hex_data = ''.join(f'{b:02x}' for b in data_buffer)
                
                address, function_code, payload = extract_first_valid_modbus_frame(hex_data)
                if address is not None and function_code is not None and payload is not None:
                    frames_processed += 1
                    print(f"\n🔍 Frame - Adresse: {address}, Funktionscode: {function_code:#04x}, Payload-Länge: {len(payload)} Bytes")
                    
                    # Unterscheide zwischen Request und Response
                    if address == 0x9F and function_code in (0x03, 0x04) and len(payload) == 4:
                        # Es ist ein Request
                        startreg = int.from_bytes(payload[0:2], byteorder='big')
                        regcount = int.from_bytes(payload[2:4], byteorder='big')
                        print(f"➡️  Modbus-Request: Startregister=0x{startreg:04X}, Registeranzahl={regcount}")
                        
                        # Speichere Request-Informationen
                        last_request = {
                            'address': address,
                            'function_code': function_code,
                            'startreg': startreg,
                            'regcount': regcount,
                            'timestamp': time.time()
                        }
                        
                        # Entferne den verarbeiteten Request aus dem Puffer (8 Bytes für einen kompletten Request)
                        if len(data_buffer) >= 8:
                            del data_buffer[:8]
                    
                    elif address != 0x9F and function_code in (0x03, 0x04) and len(payload) >= 4:
                        # Es ist eine Response
                        print(f"⬅️  Modbus-Response: Datenlänge={len(payload)} Bytes")
                        
                        # Verarbeite die Payload als Float-Werte
                        values = process_modbus_payload(payload, debug=True)
                        
                        if values and last_request:
                            startreg = last_request.get('startreg', 0x2000)
                            print(f"  Verwende Startregister 0x{startreg:04X} aus vorherigem Request")
                            
                            # Mappe Werte auf Labels und wende Plausibilitätsprüfung an
                            mapped_values = map_values_to_labels(values, startreg)
                            mapped_values = apply_plausibility_check(mapped_values)
                            
                            print("\n📨 Werte (mit Plausibilitätsprüfung):")
                            for k, v in mapped_values.items():
                                if k.startswith("Register"):
                                    # Generisches Register ohne Einheit
                                    print(f"  - {k}: {v}")
                                else:
                                    # Bekanntes Label mit Einheit
                                    unit = UNITS.get(k, "")
                                    print(f"  - {k}: {v} {unit}")
                            
                            # Sende Werte per MQTT
                            send_mqtt(mapped_values)
                        
                        # Entferne die verarbeitete Response aus dem Puffer
                        # Response-Länge: 1 (Adresse) + 1 (Funktionscode) + 1 (Byte-Count) + len(payload) + 2 (CRC)
                        frame_len = 1 + 1 + 1 + len(payload) + 2
                        if len(data_buffer) >= frame_len:
                            del data_buffer[:frame_len]
                    
                    else:
                        # Unbekannter Frame-Typ
                        print(f"⚠️ Unbekannter Frame-Typ: Adresse={address:#04x}, Funktion={function_code:#04x}")
                        # Entferne ein Byte, um im nächsten Durchlauf weiter zu suchen
                        if data_buffer:
                            del data_buffer[0]
            
            # Suche direkt nach Slave-Responses im Puffer, ohne auf Requests zu warten
            if frames_processed == 0 and len(responses_found) == 0:
                print("\n🔍 Direkte Suche nach Slave-Responses im Puffer...")
                
                # Gehe den Puffer durch und suche nach typischen Slave-Response-Mustern
                i = 0
                while i < len(data_buffer) - 5:  # Mindestens 5 Bytes für eine minimale Response
                    # Prüfe auf typische Slave-Adresse (nicht 0x9F) mit Funktionscode 0x03/0x04
                    if data_buffer[i] != 0x9F and data_buffer[i+1] in (0x03, 0x04):
                        # Versuche, eine Response zu extrahieren und zu verarbeiten
                        success, resp_length, mapped_values = extract_and_process_response(
                            data_buffer, i, data_buffer[i+1], last_request, debug=True
                        )
                        
                        if success and mapped_values:
                            print("\n📨 Werte (mit Plausibilitätsprüfung):")
                            for k, v in mapped_values.items():
                                if k.startswith("Register"):
                                    # Generisches Register ohne Einheit
                                    print(f"  - {k}: {v}")
                                else:
                                    # Bekanntes Label mit Einheit
                                    unit = UNITS.get(k, "")
                                    print(f"  - {k}: {v} {unit}")
                            
                            # Sende Werte per MQTT
                            send_mqtt(mapped_values)
                            
                            frames_processed += 1
                            
                            # Entferne die verarbeitete Response aus dem Puffer
                            if i + resp_length <= len(data_buffer):
                                del data_buffer[:i + resp_length]
                            else:
                                del data_buffer[:i]  # Teilweise Entfernung, wenn nicht genug Bytes übrig sind
                            
                            break  # Beende die Schleife nach erfolgreicher Verarbeitung
            
            # Wenn keine Frames verarbeitet wurden und der Puffer zu groß ist, kürze ihn
            if frames_processed == 0 and len(data_buffer) > MAX_BUFFER_SIZE / 2:
                # Entferne die älteste Hälfte der Daten
                print(f"⚠️ Keine Frames verarbeitet. Puffer wird gekürzt: {len(data_buffer)} -> {len(data_buffer)//2} Bytes")
                data_buffer = data_buffer[len(data_buffer)//2:]
            
            # Spezielle Taktik: Wenn sehr viele Requests gefunden werden, aber keine Responses,
            # versuche eine spezielle Suche nach Slave-Responses mit möglichen Geräte-IDs
            if frames_processed == 0 and len(requests_found) > 2 and len(responses_found) == 0:
                print("\n⚠️ Viele Requests gefunden, aber keine Responses. Versuche spezielle Suche...")
                
                # Extrahiere den neuesten Request für die Response-Korrelation
                last_req = requests_found[-1]
                
                # Typische Slave-Geräte-IDs für den DTSU666 und andere Modbus-Geräte
                for slave_id in range(1, 10):  # Suche im erweiterten Bereich von 1-9
                    # Suche im Puffer nach dieser Geräte-ID
                    i = 0
                    while i < len(data_buffer) - 5:
                        if data_buffer[i] == slave_id:
                            # Zeige die nächsten Bytes für Debug-Zwecke
                            next_bytes = data_buffer[i:i+min(20, len(data_buffer)-i)]
                            print(f"DEBUG: Potenzielle Slave-ID {slave_id} bei Position {i}: {next_bytes.hex()}")
                            
                            # Prüfe auf Funktionscode (normale Antwort oder Fehlerantwort)
                            if (i+1 < len(data_buffer) and 
                                (data_buffer[i+1] in (0x03, 0x04) or 
                                 data_buffer[i+1] in (0x83, 0x84))):
                                
                                print(f"DEBUG: Gefunden - Slave-ID {slave_id}, Funktionscode {data_buffer[i+1]:02X}")
                                
                                # Versuche, eine Response zu extrahieren und zu verarbeiten
                                function_code = data_buffer[i+1] & 0x7F  # Entferne das Fehlerbit
                                success, resp_length, mapped_values = extract_and_process_response(
                                    data_buffer, i, function_code, last_req, debug=True
                                )
                                
                                if success:
                                    print(f"\n✅ Response von Slave-ID {slave_id} erfolgreich erkannt!")
                                    
                                    if mapped_values:
                                        print("\n📨 Werte (mit Plausibilitätsprüfung):")
                                        for k, v in mapped_values.items():
                                            if k.startswith("Register"):
                                                print(f"  - {k}: {v}")
                                            else:
                                                unit = UNITS.get(k, "")
                                                print(f"  - {k}: {v} {unit}")
                                        
                                        # Sende Werte per MQTT
                                        send_mqtt(mapped_values)
                                    else:
                                        print("ℹ️ Response erkannt, aber keine Werte extrahiert (möglicherweise Fehlerantwort)")
                                    
                                    frames_processed += 1
                                    
                                    # Entferne die verarbeitete Response aus dem Puffer
                                    if i + resp_length <= len(data_buffer):
                                        del data_buffer[:i + resp_length]
                                    else:
                                        del data_buffer[:i]
                                    
                                    break  # Breche die Schleife nach erfolgreicher Verarbeitung ab
                        
                        i += 1
                    
                    if frames_processed > 0:
                        break  # Breche die Geräte-ID-Schleife ab, wenn ein Frame verarbeitet wurde
                
                # Wenn auch keine typischen Slave-IDs gefunden wurden, suche nach unbekannten Formaten
                if frames_processed == 0:
                    print("\n⚠️ Keine bekannten Slave-IDs gefunden. Suche nach alternativen Formaten...")
                    
                    # Versuche, große zusammenhängende Datenblöcke zu finden
                    i = 0
                    while i < len(data_buffer) - 10:
                        # Suche nach Blöcken, die keine Protokollmarker enthalten
                        block_start = i
                        continuous_bytes = 0
                        protocol_marker = False
                        
                        while i < len(data_buffer) - 1:
                            # Prüfe auf Protokollmarker (9F03, 9F04, etc.)
                            if data_buffer[i] == 0x9F and data_buffer[i+1] in (0x03, 0x04):
                                protocol_marker = True
                                break
                            continuous_bytes += 1
                            i += 1
                        
                        # Wenn wir einen großen zusammenhängenden Block gefunden haben
                        if continuous_bytes >= 32 and not protocol_marker:
                            block = data_buffer[block_start:block_start + continuous_bytes]
                            print(f"\nℹ️ Großer Datenblock gefunden: {continuous_bytes} Bytes")
                            print(f"  Block-Anfang: {block[:min(20, len(block))].hex()}")
                            
                            # Versuche, den Block als Float-Daten zu interpretieren
                            if continuous_bytes % 4 == 0:
                                print("  Versuche als 32-Bit-Float-Daten zu interpretieren (DTSU666-Format)...")
                                # Der DTSU666 verwendet ausschließlich Float32 für alle Register
                                values = process_modbus_payload(block, debug=True, force_format='float32')
                                
                                if values and any(not math.isnan(v) and not math.isinf(v) and v != 0 for v in values):
                                    print("  ✅ Plausible Float-Werte gefunden. DTSU666 verwendet Float32-Format.")
                                    # Versuche, die Werte zu mappen (als wären sie ab Register 0x2000)
                                    mapped_values = map_values_to_labels(values, 0x2000)
                                    mapped_values = apply_plausibility_check(mapped_values)
                                    
                                    print("\n📨 Potenzielle Werte (experimentell):")
                                    for k, v in mapped_values.items():
                                        if k.startswith("Register"):
                                            print(f"  - {k}: {v}")
                                        else:
                                            unit = UNITS.get(k, "")
                                            print(f"  - {k}: {v} {unit}")
                                    
                                    # Wir entfernen den Block nicht aus dem Puffer, da wir uns nicht sicher sind
                            
                            # Versuche, den Block als 16-Bit-Daten zu interpretieren
                            if continuous_bytes % 2 == 0:
                                print("  Versuche als 16-Bit-Integer-Daten zu interpretieren...")
                                int_values = []
                                for j in range(0, min(40, continuous_bytes), 2):
                                    if j + 2 <= len(block):
                                        val = int.from_bytes(block[j:j+2], byteorder='big')
                                        int_values.append(val)
                                
                                print(f"  Erste 16-Bit-Werte: {int_values[:10]}")
                        
                        # Gehe zum nächsten potenziellen Block
                        i = block_start + continuous_bytes + 1
            
            # Zeige Verarbeitungsstatistik
            if frames_processed > 0:
                print(f"\n✅ {frames_processed} Frames verarbeitet. Verbleibender Puffer: {len(data_buffer)} Bytes")
            else:
                print(f"\n⚠️ Keine Frames verarbeitet. Puffer: {len(data_buffer)} Bytes")
            
            # Kurze Pause
            time.sleep(0.1)

        except Exception as e:
            print(f"❌ Fehler in der Hauptschleife: {e}")
            import traceback
            traceback.print_exc()
            # Kurze Pause nach einem Fehler
            time.sleep(1)
