import serial
import time

# Berechnet das CRC16 für Modbus RTU (little endian)
def calc_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            lsb = crc & 0x0001
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc.to_bytes(2, byteorder='little')

# Dekodiert ein vollständiges Modbus-Frame
def decode_modbus_frame(frame):
    addr = frame[0]
    func = frame[1]
    data_len = frame[2]
    data = frame[3:3 + data_len]
    crc_received = frame[-2:]

    crc_calculated = calc_crc(frame[:-2])

    if crc_received != crc_calculated:
        print("❌ CRC-Fehler, verwerfe Frame.")
        return

    print(f"📨 Gültiges Frame empfangen:")
    print(f"  ➤ Adresse:        {addr}")
    print(f"  ➤ Funktion:       {func}")
    print(f"  ➤ Daten (Hex):    {data.hex()}")
    print(f"  ➤ CRC korrekt ✅")

# Findet ein gültiges Frame im Puffer
def find_valid_frame(buffer):
    for i in range(len(buffer) - 4):  # mindestens 5 Byte nötig
        if i + 3 >= len(buffer):
            break
        data_len = buffer[i + 2]
        frame_len = 3 + data_len + 2
        if i + frame_len > len(buffer):
            continue

        candidate = buffer[i:i + frame_len]
        if candidate[-2:] == calc_crc(candidate[:-2]):
            return candidate, i, frame_len
    return None, None, None

# Liest den seriellen Port und sucht nach Modbus RTU Frames
def sniff_serial(port='/dev/ttyUSB0', baudrate=9600):
    try:
        with serial.Serial(port, baudrate=baudrate, timeout=0.1) as ser:
            print(f"📡 Sniffen auf {port} @ {baudrate} Baud...")
            buffer = bytearray()
            while True:
                data = ser.read(256)
                if data:
                    buffer.extend(data)

                while len(buffer) >= 5:
                    frame, start_idx, frame_len = find_valid_frame(buffer)
                    if frame:
                        decode_modbus_frame(frame)
                        del buffer[:start_idx + frame_len]
                    else:
                        # Keine gültigen Frames mehr → erstes Byte löschen (sync shift)
                        buffer.pop(0)
                        break
    except serial.SerialException as e:
        print(f"❌ Serieller Fehler: {e}")
    except KeyboardInterrupt:
        print("⛔️ Sniffer gestoppt.")

if __name__ == '__main__':
    sniff_serial()

