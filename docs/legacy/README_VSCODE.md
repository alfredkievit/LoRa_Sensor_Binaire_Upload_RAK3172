# RAK3172 VS Code Setup

Deze workspace is ingericht voor de VS Code Arduino-extensie op de RAK RUI BSP.

Lokale board package die op deze pc is gevonden:
- `rak_rui:stm32` versie `4.2.4`

Board en configuratie in deze workspace:
- board: `rak_rui:stm32:WisDuoRAK3172EvaluationBoard`
- LoRa support: `Support LoRaWAN`
- region support: alleen `EU868`
- build output: `build/`

Aanbevolen werkwijze:
1. Open deze map in VS Code.
2. Installeer de extensie `Arduino` als die nog ontbreekt.
3. Schakel `PlatformIO` voor deze workspace uit. RAK documenteert dat de Arduino-extensie en PlatformIO elkaar hier kunnen storen.
4. Selecteer de seriele poort van de RAK3172 in VS Code.
5. Build en upload de sketch `RAK3172_Binaire_payload_rui/RAK3172_Binaire_payload_rui.ino`.

Secrets:
- `SECRET_DEV_EUI` hoort overeen te komen met de DevEUI van het module-sticker of de eerder werkende firmware.
- Werk met `RAK3172_Binaire_payload_rui/arduino_secrets.h` voor echte OTAA-sleutels.
- Gebruik `RAK3172_Binaire_payload_rui/arduino_secrets.example.h` als template voor nieuwe deployments.

Diagnosegedrag in de sketch:
- `JOIN_DIAGNOSTICS_ONLY` zet join-diagnose aan zonder sensorsend.
- `ENABLE_LOW_POWER_SLEEP` staat standaard uit zodat join-logs zichtbaar blijven.
