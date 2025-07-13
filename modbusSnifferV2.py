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
        if len(frame) < 3:
            return result
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
        
        result['data_len'] = data_len
        result['registers'] = registers
    
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
    print(f"RAW: {frame_info['raw']}")
    
    # Zusätzliche Informationen je nach Funktionscode
    if frame_info['function_code'] == 3 and 'registers' in frame_info:
        print("Register-Werte:")
        for i, reg in enumerate(frame_info['registers']):
            print(f"  Register {i}: {reg} (0x{reg:04X})")
    
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


def main():
    try:
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
                        print_frame_info(frame_info)
                        
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

if __name__ == '__main__':
    main()
