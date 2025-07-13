# Basis-Image mit Python 3
FROM python:3.9-slim

# Arbeitsverzeichnis im Container
WORKDIR /app

# Kopiere alle Dateien aus dem lokalen Projekt in das Container-Verzeichnis
COPY . /app

# Installiere die benötigten Python-Bibliotheken
RUN pip install --no-cache-dir -r requirements.txt

# Führe den Sniffer beim Start des Containers aus
ENTRYPOINT ["python", "modbusSnifferV2.py"]
