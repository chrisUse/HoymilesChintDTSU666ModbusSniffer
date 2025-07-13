import serial
import time
import binascii
import struct
import datetime

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
TIMEOUT = 0.1  # Sekunden

# Modbus RTU Frame Mindestlänge: Adresse(1) + Funktion(1) + Daten(>=2) + CRC(2)
MIN_FRAME_SIZE = 6
MAX_FRAME_SIZE = 256  # Maximale Größe eines Modbus RTU Frames

# CHINT G DTSU666 Smart Meter Register-Adressen und Formate
# Basierend auf der Dokumentation ab Seite 11
REGISTER_MAP = {
    # Spannung und Strom
    0x2000: {"name": "Spannung Phase A", "unit": "V", "factor": 0.1, "format": "float32"},
    0x2002: {"name": "Spannung Phase B", "unit": "V", "factor": 0.1, "format": "float32"},
    0x2004: {"name": "Spannung Phase C", "unit": "V", "factor": 0.1, "format": "float32"},
    0x2006: {"name": "Strom Phase A", "unit": "A", "factor": 0.1, "format": "float32"},
    0x2008: {"name": "Strom Phase B", "unit": "A", "factor": 0.1, "format": "float32"},
    0x200A: {"name": "Strom Phase C", "unit": "A", "factor": 0.1, "format": "float32"},
    
    # Leistung
    0x2014: {"name": "Wirkleistung Gesamt", "unit": "W", "factor": 0.1, "format": "float32"},
    0x2016: {"name": "Wirkleistung Phase A", "unit": "W", "factor": 0.1, "format": "float32"},
    0x2018: {"name": "Wirkleistung Phase B", "unit": "W", "factor": 0.1, "format": "float32"},
    0x201A: {"name": "Wirkleistung Phase C", "unit": "W", "factor": 0.1, "format": "float32"},
    0x201C: {"name": "Scheinleistung Gesamt", "unit": "VA", "factor": 1.0, "format": "float32"},
    0x201E: {"name": "Scheinleistung Phase A", "unit": "VA", "factor": 1.0, "format": "float32"},
    0x2020: {"name": "Scheinleistung Phase B", "unit": "VA", "factor": 1.0, "format": "float32"},
    0x2022: {"name": "Scheinleistung Phase C", "unit": "VA", "factor": 1.0, "format": "float32"},
    0x2024: {"name": "Blindleistung Gesamt", "unit": "var", "factor": 1.0, "format": "float32"},
    0x2026: {"name": "Blindleistung Phase A", "unit": "var", "factor": 1.0, "format": "float32"},
    0x2028: {"name": "Blindleistung Phase B", "unit": "var", "factor": 1.0, "format": "float32"},
    0x202A: {"name": "Blindleistung Phase C", "unit": "var", "factor": 1.0, "format": "float32"},
    
    # Leistungsfaktor
    0x202C: {"name": "Leistungsfaktor Gesamt", "unit": "", "factor": 0.001, "format": "float32"},
    0x202E: {"name": "Leistungsfaktor Phase A", "unit": "", "factor": 0.001, "format": "float32"},
    0x2030: {"name": "Leistungsfaktor Phase B", "unit": "", "factor": 0.001, "format": "float32"},
    0x2032: {"name": "Leistungsfaktor Phase C", "unit": "", "factor": 0.001, "format": "float32"},
    
    # Frequenz
    0x2044: {"name": "Frequenz", "unit": "Hz", "factor": 0.01, "format": "float32"},
    
    # Energiezähler
    0x4000: {"name": "Wirkenergie Import (+)", "unit": "kWh", "factor": 0.01, "format": "float32"},
    0x4004: {"name": "Wirkenergie Export (-)", "unit": "kWh", "factor": 0.01, "format": "float32"},
    0x4008: {"name": "Blindenergie Import (+)", "unit": "kvarh", "factor": 0.01, "format": "float32"},
    0x400C: {"name": "Blindenergie Export (-)", "unit": "kvarh", "factor": 0.01, "format": "float32"},
    
    # Maximalwerte
    0x4800: {"name": "Max. Wirkleistung Import", "unit": "W", "factor": 1.0, "format": "float32"},
    0x4804: {"name": "Max. Wirkleistung Export", "unit": "W", "factor": 1.0, "format": "float32"},
}


def is_valid_crc(frame):
    if len(frame) < MIN_FRAME_SIZE:
        return False
    data = frame[:-2]
    crc_received = frame[-2:]
    crc_calc = crc16(data)
    return crc_received == crc_calc


def crc16(data: bytes):
    '''Berechnet Modbus CRC16'''
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')


def decode_modbus_frame(frame):
    """Dekodiert einen Modbus RTU Frame und gibt die Informationen zurück."""
    if len(frame) < MIN_FRAME_SIZE:
        return None
    
    slave_addr = frame[0]
    function_code = frame[1]
    
    result = {
        'slave_addr': slave_addr,
        'function_code': function_code,
        'timestamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        'raw': binascii.hexlify(frame).decode()
    }
    
    # Funktion 3: Read Holding Registers
    if function_code == 3:
        # Beim Request
        if len(frame) < 8:  # Anfrage hat typischerweise 8 Bytes: [Addr][FC][RegH][RegL][CountH][CountL][CRCH][CRCL]
            if len(frame) >= 6:  # Anfrage mit StartAddr und Anzahl
                start_addr = (frame[2] << 8) + frame[3]
                reg_count = (frame[4] << 8) + frame[5]
                result['request_type'] = 'request'
                result['start_addr'] = start_addr
                result['reg_count'] = reg_count
                
                # Speichere diese Anfrage für die nächste Antwort
                global last_request_start_addr, last_request_registers
                last_request_start_addr = start_addr
                last_request_registers = reg_count
            return result
        
        # Bei der Antwort
        data_len = frame[2]
        if len(frame) < 3 + data_len:
            return result
        
        data = frame[3:3+data_len]
        registers = []
        
        # Daten in 16-bit Register umwandeln
        for i in range(0, len(data), 2):
            if i+1 < len(data):
                register_value = (data[i] << 8) + data[i+1]
                registers.append(register_value)
        
        result['request_type'] = 'response'
        result['data_len'] = data_len
        result['registers'] = registers
        
        # Versuche, die Register basierend auf der letzten Anfrage zu interpretieren
        if last_request_start_addr is not None:
            try:
                result['smart_meter_values'] = decode_smart_meter_registers(registers, last_request_start_addr)
            except Exception as e:
                # Bei Fehlern bei der Dekodierung, versuche es ohne Startadresse
                print(f"Fehler bei Dekodierung mit bekannter Startadresse: {e}")
                result['smart_meter_values'] = decode_smart_meter_registers(registers)
            
            # Anfrage als verarbeitet markieren, wenn die Anzahl der Register übereinstimmt
            if last_request_registers is not None and len(registers) == last_request_registers:
                last_request_start_addr = None
                last_request_registers = None
    
    # Funktion 16: Write Multiple Registers
    elif function_code == 16:
        if len(frame) < 6:
            return result
        start_addr = (frame[2] << 8) + frame[3]
        reg_count = (frame[4] << 8) + frame[5]
        
        result['start_addr'] = start_addr
        result['reg_count'] = reg_count
    
    return result


def print_frame_info(frame_info):
    """Formatierte Ausgabe der Frame-Informationen"""
    if not frame_info:
        return
    
    print(f"\n--- MODBUS FRAME [{frame_info['timestamp']}] ---")
    print(f"Slave-Adresse: {frame_info['slave_addr']}")
    print(f"Funktionscode: {frame_info['function_code']} ({get_function_name(frame_info['function_code'])})")
    
    # Bei Anfragen zusätzliche Informationen anzeigen
    if 'request_type' in frame_info and frame_info['request_type'] == 'request':
        addr_hex = f"0x{frame_info.get('start_addr', 0):04X}"
        print(f"Anfrage: Register-Adresse {addr_hex}, Anzahl: {frame_info.get('reg_count', 0)}")
        
        # Wenn bekannte Register angefragt werden, zeige die Namen an
        start_addr = frame_info.get('start_addr')
        if start_addr in REGISTER_MAP:
            print(f"Angeforderte Werte: {REGISTER_MAP[start_addr]['name']}")
    
    print(f"RAW: {frame_info['raw']}")
    
    # Zusätzliche Informationen je nach Funktionscode
    if frame_info['function_code'] == 3 and 'registers' in frame_info:
        # Nur bei vollständigen Antworten Register anzeigen
        if frame_info.get('request_type') == 'response':
            # Rohe Register-Werte anzeigen
            if len(frame_info['registers']) <= 20:  # Bei vielen Registern nicht alle anzeigen
                print("Register-Werte:")
                for i, reg in enumerate(frame_info['registers']):
                    print(f"  Register {i}: {reg} (0x{reg:04X})")
            else:
                print(f"Register-Werte: {len(frame_info['registers'])} Register empfangen")
            
            # Interpretierte Smart Meter Werte anzeigen
            if 'smart_meter_values' not in frame_info or not frame_info['smart_meter_values']:
                # Wenn keine Werte vom ersten Dekodierungsversuch, versuche es ohne Startadresse
                smart_meter_values = decode_smart_meter_registers(frame_info['registers'])
                if smart_meter_values:
                    frame_info['smart_meter_values'] = smart_meter_values
            
            if 'smart_meter_values' in frame_info and frame_info['smart_meter_values']:
                print("\nInterpretierte Smart Meter Werte:")
                
                # Gruppierte Ausgabe für bessere Übersicht
                grouped_values = {
                    "Spannungen": {},
                    "Ströme": {},
                    "Leistungen": {},
                    "Energie": {},
                    "Sonstige": {}
                }
                
                for name, info in frame_info['smart_meter_values'].items():
                    if "Spannung" in name:
                        grouped_values["Spannungen"][name] = info
                    elif "Strom" in name:
                        grouped_values["Ströme"][name] = info
                    elif any(word in name for word in ["Wirkleistung", "Scheinleistung", "Blindleistung"]):
                        grouped_values["Leistungen"][name] = info
                    elif any(word in name for word in ["energie", "Energie"]):
                        grouped_values["Energie"][name] = info
                    else:
                        grouped_values["Sonstige"][name] = info
                
                # Ausgabe der gruppierten Werte
                for group, values in grouped_values.items():
                    if values:
                        print(f"\n  -- {group} --")
                        for name, info in values.items():
                            print(f"  {name}: {info['value']:.3f} {info['unit']} (Raw: {info['raw']})")
    
    elif frame_info['function_code'] == 16:
        print(f"Start-Adresse: {frame_info.get('start_addr')}")
        print(f"Anzahl Register: {frame_info.get('reg_count')}")


def get_function_name(code):
    """Gibt den Namen des Modbus-Funktionscodes zurück"""
    function_codes = {
        1: "Read Coils",
        2: "Read Discrete Inputs",
        3: "Read Holding Registers",
        4: "Read Input Registers",
        5: "Write Single Coil",
        6: "Write Single Register",
        15: "Write Multiple Coils",
        16: "Write Multiple Registers"
    }
    return function_codes.get(code, "Unbekannt")


def find_frame_start(buffer):
    """Sucht nach möglichen Frame-Starts im Buffer"""
    # Einfache Heuristik: Ein Frame beginnt oft mit der Slave-Adresse (meist 1-247)
    # und einem gültigen Funktionscode (1-127)
    for i in range(len(buffer) - 1):
        if (1 <= buffer[i] <= 247) and (1 <= buffer[i+1] <= 127):
            return i
    return -1


def interpret_float32(high_word, low_word):
    """
    Interpretiert zwei 16-bit Register als Float32 (IEEE 754) Wert.
    CHINT G DTSU666 verwendet die Reihenfolge High-Low für 32-bit Werte.
    """
    try:
        value_bytes = high_word.to_bytes(2, 'big') + low_word.to_bytes(2, 'big')
        return struct.unpack('>f', value_bytes)[0]
    except Exception:
        # Alternative Byte-Reihenfolge probieren (für verschiedene Modbus-Implementierungen)
        try:
            value_bytes = high_word.to_bytes(2, 'little') + low_word.to_bytes(2, 'little')
            return struct.unpack('<f', value_bytes)[0]
        except Exception:
            return 0.0


def decode_smart_meter_registers(registers, request_addr=None):
    """
    Interpretiert Register-Werte basierend auf der CHINT G DTSU666 Register-Map.
    
    Args:
        registers: Liste der gelesenen Register-Werte
        request_addr: Optional - Startadresse der Anfrage, falls bekannt
    
    Returns:
        Dictionary mit interpretierten Werten
    """
    if not registers or len(registers) < 2:
        return {}
    
    # Wenn die Anfrage-Adresse nicht bekannt ist, versuchen wir sie zu erkennen
    if request_addr is None:
        # Typische Muster für Spannungs-/Strom-Daten (0x2000) erkennen
        # CHINT DTSU666 liefert typische Werte für Spannung: 220-240V (bei 0x2000)
        if len(registers) >= 2 and 17000 < registers[0] < 18000:
            request_addr = 0x2000
        # Typische Muster für Energiezähler (0x4000) erkennen
        elif len(registers) >= 2 and registers[0] > 30000:
            request_addr = 0x4000
        # Fallback: Wir probieren beide bekannten Startadressen
        else:
            # Versuche zuerst die Spannungs/Strom-Register zu dekodieren
            result_2000 = try_decode_with_addr(registers, 0x2000)
            # Dann die Energiezähler-Register
            result_4000 = try_decode_with_addr(registers, 0x4000)
            
            # Verwende das Ergebnis mit mehr erkannten Werten
            if len(result_4000) > len(result_2000):
                return result_4000
            return result_2000
    
    return try_decode_with_addr(registers, request_addr)


def try_decode_with_addr(registers, request_addr):
    """
    Versucht Register mit einer bestimmten Startadresse zu dekodieren.
    
    Args:
        registers: Liste der Register-Werte
        request_addr: Startadresse für die Interpretation
    
    Returns:
        Dictionary mit interpretierten Werten
    """
    decoded = {}
    
    # Interpretiere die Register als 32-bit Werte (jeweils 2 Register)
    for i in range(0, len(registers) - 1, 2):
        reg_addr = request_addr + i
        if reg_addr in REGISTER_MAP:
            reg_info = REGISTER_MAP[reg_addr]
            high_word = registers[i]
            low_word = registers[i + 1]
            
            if reg_info["format"] == "float32":
                try:
                    value = interpret_float32(high_word, low_word)
                    # Anwendung des Faktors für korrekte Einheit
                    value = value * reg_info["factor"]
                    
                    # Plausibilitätsprüfung für einige bekannte Werte
                    if "Spannung" in reg_info["name"] and (value < 10 or value > 500):
                        continue  # Unplausible Spannung
                    if "Strom" in reg_info["name"] and (value < 0 or value > 100):
                        continue  # Unplausible Stromstärke
                    if "Frequenz" in reg_info["name"] and (value < 45 or value > 65):
                        continue  # Unplausible Frequenz
                    
                    decoded[reg_info["name"]] = {
                        "value": value,
                        "unit": reg_info["unit"],
                        "raw": f"0x{high_word:04X}{low_word:04X}"
                    }
                except Exception:
                    # Fehlgeschlagene Float-Interpretation überspringen
                    continue
    
    return decoded


def export_to_csv(frame_info, filename="smart_meter_data.csv"):
    """
    Exportiert die interpretierten Smart Meter Werte in eine CSV-Datei.
    
    Args:
        frame_info: Das Frame-Info-Dictionary mit den Smart Meter Werten
        filename: Der Dateiname für die CSV-Datei
    """
    if not frame_info or 'smart_meter_values' not in frame_info or not frame_info['smart_meter_values']:
        return
    
    import os
    import csv
    
    # Prüfen, ob die Datei bereits existiert, um Header zu schreiben
    file_exists = os.path.isfile(filename)
    
    # Dictionary mit allen Werten für diese Zeile erstellen
    row_data = {
        'timestamp': frame_info['timestamp']
    }
    
    # Alle Smart Meter Werte zum Dictionary hinzufügen
    for name, info in frame_info['smart_meter_values'].items():
        row_data[name] = info['value']
    
    # CSV-Datei öffnen und Daten schreiben
    with open(filename, mode='a', newline='') as csvfile:
        # Alle Spaltennamen bestimmen
        fieldnames = ['timestamp'] + list(frame_info['smart_meter_values'].keys())
        
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        # Header nur schreiben, wenn die Datei neu ist
        if not file_exists:
            writer.writeheader()
        
        # Daten schreiben
        writer.writerow(row_data)


def main():
    try:
        global last_request_start_addr
        last_request_start_addr = None
        
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        print(f'Sniffe Modbus RTU auf {SERIAL_PORT} mit {BAUDRATE} Baud...')
        print(f'Drücke STRG+C zum Beenden')
        
        buffer = b''
        last_data_time = time.time()
        
        while True:
            # Daten lesen
            data = ser.read(256)
            current_time = time.time()
            
            if data:
                buffer += data
                last_data_time = current_time
                print(".", end="", flush=True)  # Aktivitätsindikator
            
            # Wenn genug Zeit ohne neue Daten vergangen ist, pufferinhalt prüfen
            if len(buffer) > 0 and (current_time - last_data_time) > TIMEOUT:
                frame_start = 0
                
                # Versuche, Frames im Buffer zu finden
                while frame_start < len(buffer) and len(buffer) - frame_start >= MIN_FRAME_SIZE:
                    # Versuche verschiedene Framegrößen
                    valid_frame = None
                    
                    for frame_size in range(MIN_FRAME_SIZE, min(MAX_FRAME_SIZE, len(buffer) - frame_start + 1)):
                        possible_frame = buffer[frame_start:frame_start+frame_size]
                        if is_valid_crc(possible_frame):
                            valid_frame = possible_frame
                            break
                    
                    if valid_frame:
                        # Frame gefunden und dekodieren
                        frame_info = decode_modbus_frame(valid_frame)
                        
                        # Wenn es sich um eine Read Holding Register Anfrage handelt, 
                        # speichere die Startadresse für die nächste Antwort
                        if (frame_info['function_code'] == 3 and
                            'request_type' in frame_info and 
                            frame_info['request_type'] == 'request'):
                            last_request_start_addr = frame_info.get('start_addr')
                        
                        print_frame_info(frame_info)
                        export_to_csv(frame_info)  # Exportiere die Daten nach CSV
                        
                        # Buffer nach dem Frame fortsetzen
                        buffer = buffer[frame_start + len(valid_frame):]
                        frame_start = 0
                    else:
                        # Keine gültige CRC, weiter im Buffer suchen
                        frame_start += 1
                
                # Wenn kein Frame gefunden wurde und der Buffer zu groß wird, älteren Teil verwerfen
                if len(buffer) > MAX_FRAME_SIZE * 2:
                    buffer = buffer[-MAX_FRAME_SIZE:]
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nProgram beendet durch Benutzer")
    except serial.SerialException as e:
        print(f"\nFehler beim Öffnen des seriellen Ports: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serieller Port geschlossen")


# Globale Variablen zur Speicherung der letzten Anfrage-Daten
last_request_start_addr = None
last_request_registers = None


def main():
    try:
        global last_request_start_addr, last_request_registers
        last_request_start_addr = None
        last_request_registers = None
        
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        print(f'Sniffe Modbus RTU auf {SERIAL_PORT} mit {BAUDRATE} Baud...')
        print(f'Drücke STRG+C zum Beenden')
        
        buffer = b''
        last_data_time = time.time()
        
        while True:
            # Daten lesen
            data = ser.read(256)
            current_time = time.time()
            
            if data:
                buffer += data
                last_data_time = current_time
                print(".", end="", flush=True)  # Aktivitätsindikator
            
            # Wenn genug Zeit ohne neue Daten vergangen ist, pufferinhalt prüfen
            if len(buffer) > 0 and (current_time - last_data_time) > TIMEOUT:
                frame_start = 0
                
                # Versuche, Frames im Buffer zu finden
                while frame_start < len(buffer) and len(buffer) - frame_start >= MIN_FRAME_SIZE:
                    # Versuche verschiedene Framegrößen
                    valid_frame = None
                    
                    for frame_size in range(MIN_FRAME_SIZE, min(MAX_FRAME_SIZE, len(buffer) - frame_start + 1)):
                        possible_frame = buffer[frame_start:frame_start+frame_size]
                        if is_valid_crc(possible_frame):
                            valid_frame = possible_frame
                            break
                    
                    if valid_frame:
                        # Frame gefunden und dekodieren
                        frame_info = decode_modbus_frame(valid_frame)
                        
                        # Wenn es sich um eine Read Holding Register Anfrage handelt, 
                        # speichere die Startadresse für die nächste Antwort
                        if (frame_info['function_code'] == 3 and
                            'request_type' in frame_info and 
                            frame_info['request_type'] == 'request'):
                            last_request_start_addr = frame_info.get('start_addr')
                            last_request_registers = frame_info.get('reg_count')
                        
                        print_frame_info(frame_info)
                        export_to_csv(frame_info)  # Exportiere die Daten nach CSV
                        
                        # Buffer nach dem Frame fortsetzen
                        buffer = buffer[frame_start + len(valid_frame):]
                        frame_start = 0
                    else:
                        # Keine gültige CRC, weiter im Buffer suchen
                        frame_start += 1
                
                # Wenn kein Frame gefunden wurde und der Buffer zu groß wird, älteren Teil verwerfen
                if len(buffer) > MAX_FRAME_SIZE * 2:
                    buffer = buffer[-MAX_FRAME_SIZE:]
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nProgram beendet durch Benutzer")
    except serial.SerialException as e:
        print(f"\nFehler beim Öffnen des seriellen Ports: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serieller Port geschlossen")


# Globale Variablen zur Speicherung der letzten Anfrage-Daten
last_request_start_addr = None
last_request_registers = None


if __name__ == '__main__':
    main()
