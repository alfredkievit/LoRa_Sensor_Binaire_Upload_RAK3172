# Changelog – RAK3172 LoRa Sensor Node

Alle noemenswaardige wijzigingen worden hier bijgehouden.
Formaat gebaseerd op [Keep a Changelog](https://keepachangelog.com/nl/1.0.0/).

---

## [Unreleased]

Functiepariteit met de MKR1310-firmware (`LoRa_Sensor_Binaire_Upload`, branch `fix/o2-low-readings`).
Getest op board v4, DevEUI `AC1F09FFFE1DA174`, RUI 4.2.4.

### Added
- O2-temperatuurcompensatie volgens Alphasense O2-A2 datasheet (zelfde tabel als MKR1310).
- Kalibratie in RUI user flash (`CalData`: magic, referentiespanning, kalibratietemperatuur, checksum);
  blijft behouden na reset en upload.
- Kalibratie met warm-up (min. 120 s, max. 600 s) en stabiliteitscheck (6 punten, delta <= 5 counts).
- `O2Status` met validatie; `65535` als O2-marker voor een echte sensorfout.
- Open-circuit test op PB4 onder 50 counts, plus zelftest bij debug-boot.
- Zonder BME280: O2 en batterij blijven verstuurd, `t_x10 = 0x7FFF` en `h_x10 = 0xFFFF`.
- I2C-busherstel bij boot en tweede init-poging voor de BME280.
- Batterij-afhankelijke slaaptijd (15 min / 4 u / 6 u) en cutoff onder 10 % (~60 dagen geen uplinks).
- Debugmodus gaat na 30 minuten zelf naar productie (vergeten J3-brug).

### Changed
- Pinmap gecorrigeerd naar schema v4: O2 op **PB4** (A1, OPA333-uitgang), was PB2 (J1).
- PA8 is de DONE-ingang van de TPL5110 en wordt nu altijd LOW gehouden; voorheen werd hij hoog gezet
  als "sensorvoeding", wat bij open JP2 de hele node uitschakelde. Vereist JP2 dicht.
- O2-omrekening: `V * 20,9 / V_ref`; default `V_ref` = 2,207 V (gemeten), was 1,04 V (gemeten op PB2).
- Lage O2 tot 0,0 % is een geldige meting; ondergrens geldt alleen nog voor kalibratie.
- Debugmodus: geen join en geen uplinks meer, status elke 30 s (zoals MKR1310).
- Productie-meting O2 met 8 samples (sneller, minder stroom).
- Mislukte join: slapen volgens interval i.p.v. elke 5 s opnieuw proberen.
- Ongebruikte I2C-defines op PB6/PB7 verwijderd; `Wire` gebruikt PA11/PA12.
- `.vscode/arduino.json`: poort COM6, onjuiste programmer `nrfjprog` verwijderd.
- Schema v4 bijgewerkt naar de laatste versie (labels D1/D2, R21/JP3 vervallen).
- README herschreven: pinmap, modi, kalibratie voor de klant, payload-markers, troubleshooting.
