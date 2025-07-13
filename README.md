# Modbus RTU Sniffer für DTSU666 - Zusammenfassung

Dieses Dokument enthält eine Zusammenfassung der wichtigsten Erkenntnisse und Implementierungsdetails des Modbus RTU Sniffers für den DTSU666 Stromzähler.

## Modbus-Kommunikation Grundlagen

- **Master-Request Format**: 
  - Adresse: `0x9F` (spezifisch für Master)
  - Funktionscode: `0x03` oder `0x04` (Read Holding/Input Registers)
  - Startregister: 2 Bytes (typischerweise 0x2000 - 0x2200)
  - Registeranzahl: 2 Bytes (typischerweise 1-64)
  - CRC: 2 Bytes
  - Gesamtlänge: 8 Bytes

- **Slave-Response Format**:
  - Adresse: Unterschiedlich von `0x9F` (typischerweise die Geräteadresse)
  - Funktionscode: `0x03` oder `0x04` (gleich wie im Request)
  - Byte-Count: 1 Byte (Anzahl der folgenden Datenbytes, typischerweise Vielfaches von 4)
  - Payload: Byte-Count Bytes
  - CRC: 2 Bytes
  - Gesamtlänge: 3 + Byte-Count + 2 Bytes

## Datenformat und Interpretation

- **Float-Format**: Der DTSU666 verwendet "Floating Inverse (CDAB)" Format
  - Von 4 Bytes [A, B, C, D] wird [C, D, A, B] für die Float-Dekodierung verwendet
  - Implementiert in `parse_modbus_float_inverse()`

- **Register und Werte**:
  - Startregister typischerweise bei 0x2000
  - Register enthalten Spannungen, Ströme, Leistungen, etc.
  - Einige Werte müssen mit Skalierungsfaktoren multipliziert werden (z.B. Spannungen mit 0.1)

## Implementierungsdetails

- **Frame-Extraktion**:
  - Suche nach Master-Requests (0x9F + 0x03/0x04)
  - Suche nach Slave-Responses (nicht 0x9F + 0x03/0x04 + plausible Byte-Count)
  - Zusammengehörige Request-Response-Paare identifizieren

- **Puffer-Management**:
  - Begrenzte Puffergröße (MAX_BUFFER_SIZE)
  - Verwerfen von alten Daten, wenn der Puffer zu groß wird
  - Behandlung von unvollständigen oder korrupten Frames

- **Payload-Verarbeitung**:
  - Filterung von eingebetteten Protokollmarkern
  - Konvertierung der Payload in Float-Werte
  - Plausibilitätsprüfung der Werte

- **Werte-Mapping**:
  - Zuordnung von Werten zu Labels basierend auf dem Startregister
  - Verwendung von generischen Namen für unbekannte Register
  - Anwendung von Skalierungsfaktoren und Einheiten

## Wichtige Funktionen

1. **extract_first_valid_modbus_frame**: Extrahiert den ersten gültigen Modbus-Frame aus einem Hex-String
2. **parse_modbus_float_inverse**: Parst einen 4-Byte Chunk als Float im CDAB-Format
3. **map_values_to_labels**: Mappt Werte auf Labels basierend auf dem Startregister
4. **apply_plausibility_check**: Prüft, ob die Werte physikalisch plausibel sind
5. **debug_modbus_float_variants**: Zeigt verschiedene Interpretationen eines 4-Byte-Chunks als Float
6. **process_modbus_payload**: Verarbeitet einen Modbus-Payload und extrahiert Float-Werte
7. **validate_float_block**: Überprüft, ob ein Byte-Block gültige Float-Werte enthält
8. **process_request_response_pair**: Verarbeitet ein Request-Response-Paar von Modbus-Daten

## Wichtige Erkenntnisse

1. "9F03" ist **kein** universeller Marker, sondern:
   - 0x9F ist die Master-Adresse
   - 0x03 ist der Funktionscode (Read Holding Registers)
   - Slave-Responses haben eine andere Adresse

2. Die korrekte Zuordnung von Werten zu Labels hängt vom Startregister des Requests ab.

3. Protokollmarker können in der Payload eingebettet sein und müssen gefiltert werden.

4. Die Plausibilitätsprüfung ist wichtig, um fehlerhafte Werte zu erkennen.

5. Ein gutes Puffer-Management ist entscheidend für die Verarbeitung kontinuierlicher Datenströme.

## Fehlerbehebung

- Überprüfen Sie das Float-Format, wenn Werte nicht korrekt interpretiert werden
- Stellen Sie sicher, dass Request und Response korrekt zugeordnet werden
- Achten Sie auf eingebettete Protokollmarker in der Payload
- Überprüfen Sie die Skalierungsfaktoren und Plausibilitätsbereiche

## MQTT-Integration

Der Sniffer sendet die dekodierten Werte an einen MQTT-Broker:
- Broker: 192.168.1.149:1882
- Topic: dtsu666/values
- Format: JSON mit Label/Wert-Paaren

## Debugging und Response-Erkennung

In der aktuellen Version wurden erweiterte Debugging-Funktionen und verbesserte Methoden zur Response-Erkennung implementiert:

### Neue Debugging-Funktionen
- Detaillierte Ausgabe der Bytes nach jedem Request
- Hexadezimale Darstellung der gefundenen Responses
- Analyse von potentiellen Response-Daten

### Erweiterte Response-Erkennung
- Neue Funktion `extract_and_process_response` zur direkten Verarbeitung von Responses
- Mehrschichtige Suche nach Slave-Responses:
  1. Korrelation mit vorherigen Requests
  2. Direkte Suche im gesamten Puffer
  3. Spezielle Suche nach typischen Slave-IDs (1, 2, 3)
- Flexiblere Bytecount-Validierung für robustere Response-Erkennung

### Problembehebung bei fehlenden Responses
Wenn keine Responses erkannt werden:
- Versuche die Slave-ID des DTSU666 zu ermitteln (typischerweise 1, 2 oder 3)
- Überprüfe die serielle Verbindung und Baudrate
- Aktiviere Debug-Ausgaben für detaillierte Protokollanalyse
- Prüfe, ob die Slave-Responses überhaupt im seriellen Datenstrom enthalten sind

Die Implementierung wurde darauf optimiert, auch in schwierigen Situationen Responses zu erkennen und zu verarbeiten, auch wenn die typische Request-Response-Paarung nicht immer eindeutig ist.

## Kommandozeilenoptionen und Debug-Modus

Der Modbus RTU Sniffer unterstützt verschiedene Kommandozeilenoptionen:

```bash
python modbusSnifferV2.py [OPTIONEN]
```

### Verfügbare Optionen:

- `-d, --debug`: Aktiviert den Debug-Modus für ausführliche Ausgaben
- `-p, --port PORT`: Gibt den seriellen Port an (Standard: /dev/ttyUSB0)
- `-b, --baudrate RATE`: Setzt die Baudrate (Standard: 9600)
- `-i, --interval SEKUNDEN`: Ändert das MQTT Veröffentlichungsintervall (Standard: 10 Sekunden)

### Debug-Modus

Im Debug-Modus werden detaillierte Informationen zu jedem Modbus-Frame ausgegeben, einschließlich:
- Rohe Frame-Daten
- Dekodierte Register-Werte
- Interpretierte Smart Meter Werte (Spannungen, Ströme, Leistungen, etc.)

Ohne Debug-Modus werden nur wichtige Meldungen angezeigt:
- Programmstart und -ende
- MQTT-Veröffentlichungen
- Fehlermeldungen

Beispiel zur Aktivierung des Debug-Modus:
```bash
python modbusSnifferV2.py --debug
```

Beispiel mit mehreren Optionen:
```bash
python modbusSnifferV2.py --debug --port /dev/ttyUSB1 --baudrate 19200 --interval 5
```

## Konfiguration über Umgebungsdatei (.env)

Der Modbus RTU Sniffer kann über eine `.env`-Datei konfiguriert werden. Dies ermöglicht eine einfache Anpassung der Konfiguration ohne den Quellcode zu ändern.

### Verwendung der .env-Datei

1. Kopiere die Beispieldatei `.env.example` nach `.env`:
   ```bash
   cp .env.example .env
   ```

2. Bearbeite die `.env`-Datei mit deinen Einstellungen:
   ```bash
   nano .env
   ```

3. Verfügbare Konfigurationsparameter:

   **Serielle Schnittstelle:**
   - `SERIAL_PORT`: Serieller Port (z.B. `/dev/ttyUSB0`)
   - `BAUDRATE`: Baudrate (z.B. `9600`)
   - `TIMEOUT`: Timeout in Sekunden (z.B. `0.1`)

   **MQTT-Konfiguration:**
   - `MQTT_BROKER`: IP-Adresse oder Hostname des MQTT-Brokers
   - `MQTT_PORT`: Port des MQTT-Brokers
   - `MQTT_TOPIC`: MQTT-Topic für die Daten
   - `MQTT_USERNAME`: Benutzername für MQTT (optional)
   - `MQTT_PASSWORD`: Passwort für MQTT (optional)
   - `MQTT_PUBLISH_INTERVAL`: Veröffentlichungsintervall in Sekunden

   **Debug-Modus:**
   - `DEBUG_MODE`: `true` für Debug-Ausgaben, `false` für normale Ausgaben

### Priorität der Konfiguration

Die Konfigurationsparameter werden in folgender Reihenfolge angewendet:

1. Befehlszeilenargumente (höchste Priorität)
2. Einstellungen in der `.env`-Datei
3. Standardwerte im Code (niedrigste Priorität)

### Alternative Konfigurationsdatei

Du kannst auch eine alternative Konfigurationsdatei angeben:

```bash
python modbusSnifferV2.py --env meine_config.env
```
