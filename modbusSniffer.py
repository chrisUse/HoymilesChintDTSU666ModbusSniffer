import serial
import struct
import time

def calc_crc(data: bytes) -> int:
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def parse_float32(data_bytes):
    values = []
    for i in range(0, len(data_bytes) - 3, 4):
        try:
            val = struct.unpack('>f', data_bytes[i:i+4])[0]
            values.append(val)
        except:
            values.append(None)
    return values

def try_parse_frame(buffer):
    """Suche nach gültigem Frame im laufenden Buffer."""
    for start in range(len(buffer) - 7):  # mindestens 8 Bytes
        if buffer[start+1] == 0x03:
            byte_count = buffer[start+2]
            frame_len = 3 + byte_count + 2
            if start + frame_len <= len(buffer):
                frame = buffer[start:start + frame_len]
                crc_received = int.from_bytes(frame[-2:], 'little')
                crc_calc = calc_crc(frame[:-2])
                if crc_received == crc_calc:
                    return frame, start, frame_len
    return None, None, None

def decode_frame(frame):
    addr = frame[0]
    func = frame[1]
    byte_count = frame[2]
    data = frame[3:-2]
    floats = parse_float32(data)
    print(f"\n📨 Gültiges Frame empfangen (Adresse {addr}):")
    print(f"  ➤ Daten (Hex): {data.hex()}")
    print(f"  ➤ Float-Werte:")
    for i, f in enumerate(floats):
        print(f"    - [{i}] = {f:.3f}")

def sniff(port='/dev/ttyUSB0', baudrate=9600):
    with serial.Serial(port, baudrate, timeout=1) as ser:
        print(f"📡 Sniffen auf {port} @ {baudrate} Baud...")
        buffer = bytearray()
        while True:
            data = ser.read(128)
            if data:
                buffer.extend(data)
                frame, start, length = try_parse_frame(buffer)
                if frame:
                    decode_frame(frame)
                    buffer = buffer[start+length:]  # Rest weiter analysieren
            time.sleep(0.01)

if __name__ == "__main__":
    sniff()

