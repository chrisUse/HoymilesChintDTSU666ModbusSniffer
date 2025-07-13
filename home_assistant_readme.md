# Smart Meter Integration für Home Assistant

Diese Dateien enthalten Konfigurationen, um die Daten des CHINT G DTSU666 Smart Meters in Home Assistant darzustellen.

## Voraussetzungen

1. Home Assistant muss installiert sein
2. Der MQTT-Broker muss in Home Assistant konfiguriert sein
3. Der Modbus Sniffer (`modbusSnifferV2.py`) muss ausgeführt werden und Daten an den MQTT-Broker senden

## Einrichtung

### 1. MQTT-Broker in Home Assistant konfigurieren

Falls noch nicht geschehen, füge folgende Zeilen zu deiner `configuration.yaml` hinzu:

```yaml
mqtt:
  broker: 192.168.1.149  # Passe die IP-Adresse an
  port: 1882             # Passe den Port an
  username: user1        # Passe Benutzername an
  password: user1        # Passe Passwort an
```

Verwende hier die gleichen Daten wie in der `mqtt_config` im Python-Skript.

### 2. Sensoren konfigurieren

Es gibt zwei Möglichkeiten, die Sensoren zu konfigurieren:

#### Option 1: Direkt in configuration.yaml

Kopiere den Inhalt von `smart_meter_sensors.yaml` in deine `configuration.yaml`.

#### Option 2: Mit Include (empfohlen)

1. Kopiere die Datei `smart_meter_sensors.yaml` in dein Home Assistant config-Verzeichnis
2. Füge folgende Zeile zu deiner `configuration.yaml` hinzu:

```yaml
sensor: !include smart_meter_sensors.yaml
```

### 3. Dashboard konfigurieren

Um das vordefinierte Dashboard zu nutzen:

1. Gehe in Home Assistant zu "Übersicht" → "Dashboard bearbeiten"
2. Klicke auf die drei Punkte in der oberen rechten Ecke
3. Wähle "Rohe Konfigurationseditor"
4. Ersetze den Inhalt durch den Inhalt der Datei `smart_meter_dashboard.yaml`
5. Speichere

Alternativ kannst du ein neues Dashboard erstellen und die Konfiguration dort einfügen.

## Fehlersuche

Falls die Sensoren keine Daten anzeigen:

1. Überprüfe, ob der Modbus Sniffer läuft und Daten sendet
2. Kontrolliere die MQTT-Konfiguration in Home Assistant und im Skript
3. Überprüfe mit einem MQTT-Client (z.B. MQTT Explorer), ob Daten am Topic `smartmeter/data` ankommen
4. Prüfe das Format der empfangenen Daten und vergleiche es mit den value_template-Einträgen in der Sensorkonfiguration

## Anpassung

Du kannst weitere Sensoren hinzufügen, indem du das Format in `smart_meter_sensors.yaml` kopierst und anpasst. Achte darauf, den korrekten Pfad zum Wert im JSON-Objekt zu verwenden.

Die Struktur des JSON-Objekts, das vom Modbus Sniffer gesendet wird, sieht so aus:

```json
{
  "timestamp": "2023-07-13 12:34:56.789",
  "values": {
    "Spannung Phase A": {
      "value": 230.5,
      "unit": "V",
      "raw": "0x12345678"
    },
    ...weitere Werte...
  }
}
```

## Timeout und Aktualisierungseinstellungen

Das System ist mit verschiedenen Timeout- und Aktualisierungseinstellungen konfiguriert, die die Häufigkeit der Aktualisierungen kontrollieren:

### Python-Skript (Modbus-Sniffer)

Im `modbusSnifferV2.py` ist eine Drosselung für MQTT-Veröffentlichungen implementiert:

```python
MQTT_PUBLISH_INTERVAL = 10  # Sekunden zwischen MQTT-Veröffentlichungen
```

Dies begrenzt, wie oft Daten vom Skript an MQTT gesendet werden, unabhängig davon, wie oft Modbus-Frames erfasst werden.

### Home Assistant Sensoren

In der `smart_meter_sensors.yaml` sind folgende Einstellungen für jeden Sensor aktiviert:

1. `throttle: 00:00:10` - Aktualisierung höchstens alle 10 Sekunden
2. `expire_after: 600` - Sensor wird nach 10 Minuten ohne neue Daten als "unavailable" markiert
3. `force_update: false` - Nur Aktualisieren, wenn sich der Wert ändert

Diese Einstellungen reduzieren die Systemlast und die Größe der Home Assistant-Datenbank.

### Anpassung der Aktualisierungsrate

Um die Aktualisierungsrate zu ändern:

1. Ändere `MQTT_PUBLISH_INTERVAL` im Python-Skript
2. Passe `throttle: 00:00:10` in der YAML-Datei an (Format: HH:MM:SS)

Für detaillierte Informationen siehe `timeout_settings.md`.
