# Wetter
Wetterstation Python RaspberryPi

# Service Commands for Raspi

## Status des wetter program services:
`sudo systemctl status wetter.service`

## Start (falls crash):
`sudo systemctl start wetter.service`

## Stop (zum Editiered des Python Programms Wetter.py):
`sudo systemctl stop wetter.service`

## Ausgabetext lesen:
`sudo journalctl -r -u wetter.service`

## Im Falle einer notwendingen Aenderung der System Service Definition:
### 1) Editierung der System Sevice Beschreibung:
`sudo nano /etc/systemd/system/wetter.service`

### 2) Neuladen der Beschreibung die soeben editiert wurde:
`sudo systemctl daemon-reload`

### 3) System Service neu starten:
`sudo systemctl restart wetter.service`

## Sicherung der Datenbank (nicht vergessen das Datum im Dateinamen zu ändern und die Datei vom Raspberry herunter zu kopieren):
`mysqldump -praspberry --all-databases > wetter202108171144.sql`




