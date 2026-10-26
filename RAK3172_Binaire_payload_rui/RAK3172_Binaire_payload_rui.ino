/*
  RAK3172 RUI3/RUI4 - O2 (Alphasense O2-A2) + BME280 + LoRaWAN
  Functioneel gelijk aan de MKR1310-firmware (LoRa_Sensor_Binaire_Upload).

  Target board package: rak_rui stm32 4.2.4

  2 MODES (ALLEEN bepaald door PA4 / D1 / J3 pin 4 bij boot):

  1) DEBUG + KALIBRATIE mode  (PA4 LOW binnen 2s na boot)
     - Serial prints ON (115200)
     - Direct kalibratie:
         * standaard kalibratiegas = 20.9% als je niets invoert
         * ander gas: voer O2% in via Serial binnen timeout en druk Enter
         * warm-up (>= 2 min) + stabiliteitscheck voordat er iets wordt opgeslagen
     - Slaat O2-referentie + kalibratietemperatuur op in user flash
     - Na kalibratie: GEEN join/uplink; elke 30s payload-preview + actuele waarden

  2) PRODUCTIE mode (PA4 NIET LOW bij boot)
     - Geen Serial prints
     - Join + uplink, daarna RUI sleep met batterij-afhankelijk interval:
         * <10%   => "helemaal uit": ~2 maanden slapen, geen uplink meer
         * 10-14% => ~6 uur
         * 15-19% => ~4 uur
         * >=20%  => ~15 min

  Hardware: print v4, JP2 dicht (TPL5110 overbrugd), TPL5110 DONE (PA8) blijft LOW.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "arduino_secrets.h"

// ================= CONFIG =================
constexpr uint32_t JOIN_ATTEMPT_TIMEOUT_MS   = 90UL * 1000UL;
constexpr uint32_t JOIN_RETRY_DELAY_MS       = 5UL * 1000UL;
constexpr uint32_t JOIN_POLL_INTERVAL_MS     = 2UL * 1000UL;
constexpr uint8_t  JOIN_MAX_ATTEMPTS         = 3;
constexpr uint8_t  LORA_FPORT                = 1;
constexpr bool     LORA_CONFIRMED_UPLINK     = false;
constexpr uint8_t  LORA_SEND_RETRY           = 1;
constexpr uint32_t SEND_RESULT_TIMEOUT_MS    = 15UL * 1000UL;
constexpr uint32_t DEBUG_BOOT_WINDOW_MS      = 2000UL;
constexpr uint32_t DEBUG_PAYLOAD_PREVIEW_MS  = 30UL * 1000UL;
constexpr uint32_t CAL_GAS_INPUT_TIMEOUT_MS  = 10UL * 1000UL;
// Vergeten J3-brug: na deze tijd gaat de node zelf naar productie (join + uplinks)
constexpr uint32_t DEBUG_MODE_MAX_MS         = 30UL * 60UL * 1000UL;

// ================= PINMAP (schema v4) =================
// I2C: Wire default PA11 (SDA) / PA12 (SCL)
#define BAT_ADC_PIN         PB3   // A0, VBAT divider 470k/1.5M
#define O2_ADC_PIN          PB4   // A1, OPA333 output
#define TPL_DONE_PIN        PA8   // TPL5110 DONE: HIGH = cut VBAT. Keep LOW (JP2 bridged, RUI sleep)
#define SERVICE_MODE_PIN    PA4   // D1 / J3 pin 4, LOW at boot = debug + calibration mode
#define I2C_SDA_PIN         PA11  // only used for bus recovery; Wire uses these by default
#define I2C_SCL_PIN         PA12

// ================= BME280 ADRESSEN =================
#define BME280_ADDR          0x77
#define BME280_CHIPID        0x60
#define BME280_REG_CHIPID    0xD0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_TEMP      0xFA
#define BME280_REG_HUM       0xFD

// ================= BATTERY =================
constexpr float   R_TOP = 470000.0f;
constexpr float   R_BOT = 1500000.0f;
constexpr float   VREF  = 3.3f;
constexpr uint8_t BAT_CUTOFF_PCT = 10;   // onder 10%: helemaal uit

// ================= BME280 PLAUSIBILITY / PAYLOAD MARKERS =================
// Identiek aan MKR1310 fix/o2-low-readings
constexpr float    BME_TEMP_MIN_VALID       = -40.0f;
constexpr float    BME_TEMP_MAX_VALID       = 85.0f;
constexpr float    BME_RH_MIN_VALID         = 0.0f;
constexpr float    BME_RH_MAX_VALID         = 100.0f;
constexpr int16_t  T_PAYLOAD_INVALID_MARKER = 0x7FFF;
constexpr uint16_t H_PAYLOAD_INVALID_MARKER = 0xFFFF;

// Sleep intervals (RUI sleep.all, in chunks zodat uint32 ms niet overloopt)
constexpr uint32_t SLEEP_15MIN_MS   = 15UL * 60UL * 1000UL;
constexpr uint32_t SLEEP_4H_MS      = 4UL * 60UL * 60UL * 1000UL;
constexpr uint32_t SLEEP_6H_MS      = 6UL * 60UL * 60UL * 1000UL;
constexpr uint32_t SLEEP_DAY_MS     = 24UL * 60UL * 60UL * 1000UL;
constexpr uint32_t SLEEP_CHUNK_MS   = 60UL * 1000UL;
constexpr uint16_t CUTOFF_DAYS      = 60;

// ================= O2 =================
// ADC-spanning: counts * O2_ADC_VREF / 1023. O2% = V * 20.9 / V_ref(20.9%).
constexpr float    O2_ADC_VREF                    = 3.1714f;
constexpr float    O2_SENSOR_FULL_SCALE_COUNTS    = 1023.0f;
constexpr float    O2_CAL_V_DEFAULT               = 2.207f;  // fallback: spanning bij 20.9% O2 (gemeten 2026-10-26, PB4)
constexpr float    O2_CAL_V_MIN_VALID             = 0.30f;
constexpr float    O2_CAL_V_MAX_VALID             = 2.50f;
constexpr uint32_t O2_ADC_SAMPLE_DELAY_MS         = 1;
constexpr uint8_t  O2_STABILIZED_SAMPLES          = 20;
constexpr uint32_t O2_STABILIZED_DELAY_MS         = 50;
constexpr uint8_t  O2_DEBUG_STABILIZED_READS      = 3;
constexpr uint8_t  O2_PRODUCTION_ADC_SAMPLES      = 8;
constexpr uint32_t O2_PRODUCTION_ADC_SAMPLE_DELAY_MS = 0;
constexpr float    O2_PERCENT_MIN_VALID           = 0.0f;
constexpr float    O2_PERCENT_MAX_VALID           = 25.0f;
constexpr float    O2_PERCENT_AIR_DISPLAY_CAP     = 20.0f;
constexpr float    O2_PERCENT_CAL_INPUT_MAX       = 30.0f;
constexpr float    O2_DEFAULT_CAL_GAS             = 20.9f;
constexpr uint16_t O2_PAYLOAD_INVALID_MARKER      = 65535;
// Geen ondergrens voor metingen: 0 V is een geldige 0% O2 (CA-opslag meet 0.5-3% O2).
// Een losse cel is niet te onderscheiden van 0%, want R15 trekt de opamp-ingang naar GND.
// Onder O2_ADC_LOW_CHECK wel een open-circuit test: zwevende PB4 (opamp-uitgang los) = defect.
constexpr float    O2_ADC_MAX_VALID               = 970.0f;  // verzadiging
constexpr float    O2_ADC_LOW_CHECK               = 50.0f;
constexpr uint32_t O2_OPEN_CIRCUIT_SETTLE_MS      = 10;
// Kalibratie heeft wel een bruikbaar signaal nodig (~1.5% O2 bij huidige versterking)
constexpr float    O2_CAL_ADC_MIN_VALID           = 50.0f;
constexpr float    O2_CAL_STABILITY_DELTA_MAX     = 5.0f;
constexpr uint8_t  O2_CAL_STABILITY_POINTS        = 6;
constexpr uint32_t O2_CAL_SAMPLE_INTERVAL_MS      = 10UL * 1000UL;
constexpr uint32_t O2_CAL_MIN_WARMUP_MS           = 120UL * 1000UL;
constexpr uint32_t O2_CAL_MAX_WARMUP_MS           = 600UL * 1000UL;
constexpr float    O2_CAL_DEFAULT_TEMP_C          = 20.0f;   // aangenomen temp voor default referentie

// ================= STM32 UID =================
#define UID64_BASE_ADDR 0x1FFF7580

// ================= BME280 CALIBRATIE =================
struct {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_H1;
  int16_t  dig_H2;
  uint16_t dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
  int32_t  t_fine;
} bme_cal;

// ================= PAYLOAD =================
struct __attribute__((packed)) PayloadV1 {
  uint8_t  msgType;   // = 0x01
  uint16_t o2_x10;    // O2% * 10, 65535 = invalid O2 measurement
  int16_t  t_x10;     // degC * 10
  uint16_t h_x10;     // RH% * 10
  uint8_t  bat_pct;   // 0..100
};

static_assert(sizeof(PayloadV1) == 8, "PayloadV1 must remain 8 bytes");

struct O2Reading {
  float   o2;
  float   adc;
  float   adc_min;
  float   adc_max;
  bool    valid;
  uint8_t status;
  bool    open_circuit_tested;
  bool    open_circuit;
};

enum O2Status : uint8_t {
  O2_STATUS_OK = 0,
  O2_STATUS_ADC_TOO_LOW = 1,      // only for a non-finite ADC average; low counts are valid
  O2_STATUS_ADC_TOO_HIGH = 2,
  O2_STATUS_O2_OUT_OF_RANGE = 3,
  O2_STATUS_NO_VALID_CALIBRATION = 4,
  O2_STATUS_SENSOR_DISCONNECTED = 5
};

// ================= FLASH CALIBRATION =================
struct CalData {
  uint32_t magic;
  float    o2_cal_v;
  float    cal_temp;
  uint32_t checksum;
};

constexpr uint32_t CAL_MAGIC        = 0x4F324341; // 'O2CA'
constexpr uint32_t CAL_FLASH_OFFSET = 0;          // RUI user flash partition

// ================= RUNTIME STATE =================
static bool     g_bme_ready = false;
static bool     g_debug_mode = false;
static bool     g_lora_ready = false;
static bool     g_join_announced = false;
static volatile bool    g_join_event_seen = false;
static volatile int32_t g_last_join_status = INT32_MIN;
static volatile bool    g_send_event_seen = false;
static volatile int32_t g_last_send_status = INT32_MIN;
static float    g_o2_cal_v = O2_CAL_V_DEFAULT;
static float    g_o2_cal_temp = O2_CAL_DEFAULT_TEMP_C;

#define DBG_PRINT(x) do { if (g_debug_mode) { Serial.print(x); } } while (0)
#define DBG_PRINTLN(x) do { if (g_debug_mode) { Serial.println(x); } } while (0)
#define DBG_PRINTF(...) do { if (g_debug_mode) { Serial.printf(__VA_ARGS__); } } while (0)

// ================= STATUS HELPERS =================
static const char *loramac_status_name(int32_t status) {
  switch (status) {
    case RAK_LORAMAC_STATUS_OK:                         return "OK";
    case RAK_LORAMAC_STATUS_ERROR:                      return "ERROR";
    case RAK_LORAMAC_STATUS_TX_TIMEOUT:                 return "TX_TIMEOUT";
    case RAK_LORAMAC_STATUS_RX1_TIMEOUT:                return "RX1_TIMEOUT";
    case RAK_LORAMAC_STATUS_RX2_TIMEOUT:                return "RX2_TIMEOUT";
    case RAK_LORAMAC_STATUS_RX1_ERROR:                  return "RX1_ERROR";
    case RAK_LORAMAC_STATUS_RX2_ERROR:                  return "RX2_ERROR";
    case RAK_LORAMAC_STATUS_JOIN_FAIL:                  return "JOIN_FAIL";
    case RAK_LORAMAC_STATUS_DOWNLINK_REPEATED:          return "DOWNLINK_REPEATED";
    case RAK_LORAMAC_STATUS_TX_DR_PAYLOAD_SIZE_ERROR:   return "TX_DR_PAYLOAD_SIZE_ERROR";
    case RAK_LORAMAC_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS:
      return "DOWNLINK_TOO_MANY_FRAMES_LOSS";
    case RAK_LORAMAC_STATUS_ADDRESS_FAIL:               return "ADDRESS_FAIL";
    case RAK_LORAMAC_STATUS_MIC_FAIL:                   return "MIC_FAIL";
    case RAK_LORAMAC_STATUS_MULTICAST_FAIL:             return "MULTICAST_FAIL";
    case RAK_LORAMAC_STATUS_BEACON_LOCKED:              return "BEACON_LOCKED";
    case RAK_LORAMAC_STATUS_BEACON_LOST:                return "BEACON_LOST";
    case RAK_LORAMAC_STATUS_BEACON_NOT_FOUND:           return "BEACON_NOT_FOUND";
    default:                                            return "UNKNOWN";
  }
}

static const char *region_name(int32_t region) {
  switch (region) {
    case RAK_REGION_EU868: return "EU868";
    case RAK_REGION_EU433: return "EU433";
    case RAK_REGION_US915: return "US915";
    case RAK_REGION_AU915: return "AU915";
    case RAK_REGION_AS923: return "AS923";
    case RAK_REGION_CN470: return "CN470";
    case RAK_REGION_IN865: return "IN865";
    case RAK_REGION_KR920: return "KR920";
    case RAK_REGION_RU864: return "RU864";
    default:               return "UNKNOWN";
  }
}

static const char *join_mode_name(bool otaa) {
  return otaa ? "OTAA" : "ABP";
}

static const char *network_mode_name(bool lorawan) {
  return lorawan ? "LoRaWAN" : "P2P";
}

static const char *class_name(uint8_t klass) {
  switch (klass) {
    case RAK_LORA_CLASS_A: return "Class A";
    case RAK_LORA_CLASS_B: return "Class B";
    case RAK_LORA_CLASS_C: return "Class C";
    default:               return "Unknown";
  }
}

static const char *o2_status_name(uint8_t status) {
  switch (status) {
    case O2_STATUS_OK:                   return "OK";
    case O2_STATUS_ADC_TOO_LOW:          return "ADC_TOO_LOW";
    case O2_STATUS_ADC_TOO_HIGH:         return "ADC_TOO_HIGH";
    case O2_STATUS_O2_OUT_OF_RANGE:      return "O2_OUT_OF_RANGE";
    case O2_STATUS_NO_VALID_CALIBRATION: return "NO_VALID_CALIBRATION";
    case O2_STATUS_SENSOR_DISCONNECTED:  return "SENSOR_DISCONNECTED";
    default:                             return "UNKNOWN";
  }
}

static void read_hardware_uid64(uint8_t uid[8]) {
  const uint32_t *uid64_ptr = (const uint32_t *)UID64_BASE_ADDR;
  uint32_t word0 = uid64_ptr[0];
  uint32_t word1 = uid64_ptr[1];

  uid[0] = (word0 >> 24) & 0xFF;
  uid[1] = (word0 >> 16) & 0xFF;
  uid[2] = (word0 >> 8) & 0xFF;
  uid[3] = word0 & 0xFF;
  uid[4] = (word1 >> 24) & 0xFF;
  uid[5] = (word1 >> 16) & 0xFF;
  uid[6] = (word1 >> 8) & 0xFF;
  uid[7] = word1 & 0xFF;
}

static bool eui_is_all_zero(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] != 0U) return false;
  }
  return true;
}

// ================= CALLBACKS =================
void joinCallback(int32_t status) {
  g_join_event_seen = true;
  g_last_join_status = status;
  DBG_PRINTF("[JOIN] Callback status=%ld (%s)\r\n",
             (long)status, loramac_status_name(status));
}

void sendCallback(int32_t status) {
  g_send_event_seen = true;
  g_last_send_status = status;
  DBG_PRINTF("[SEND] Callback status=%ld (%s)\r\n",
             (long)status, loramac_status_name(status));
}

// ================= HELPERS =================
static void print_hex_compact(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    DBG_PRINTF("%02X", data[i]);
  }
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static bool is_hex_string(const char *value, size_t expected_chars) {
  if (!value) return false;
  if (strlen(value) != expected_chars) return false;
  for (size_t i = 0; i < expected_chars; i++) {
    if (hex_nibble(value[i]) < 0) return false;
  }
  return true;
}

static bool hex_to_bytes(const char *hex, uint8_t *out, size_t out_len) {
  if (!hex || !out) return false;
  if (strlen(hex) != out_len * 2U) return false;

  for (size_t i = 0; i < out_len; i++) {
    int hi = hex_nibble(hex[i * 2U]);
    int lo = hex_nibble(hex[i * 2U + 1U]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

static bool validate_and_log_secrets() {
  bool ok = true;

  DBG_PRINTLN("[SECRETS] Validating JoinEUI/AppKey...");

  if (!is_hex_string(SECRET_APP_EUI, 16)) {
    DBG_PRINTLN("[SECRETS] ERROR: SECRET_APP_EUI must be exactly 16 hex chars");
    ok = false;
  } else {
    DBG_PRINTLN("[SECRETS] JoinEUI/AppEUI format OK");
  }

  if (!is_hex_string(SECRET_APP_KEY, 32)) {
    DBG_PRINTLN("[SECRETS] ERROR: SECRET_APP_KEY must be exactly 32 hex chars");
    ok = false;
  } else {
    DBG_PRINTLN("[SECRETS] AppKey format OK");
  }

  return ok;
}

static void log_rui_versions() {
  DBG_PRINTLN("[RUI] Runtime versions:");
  DBG_PRINTF("  Firmware: %s\r\n", api.system.firmwareVersion.get().c_str());
  DBG_PRINTF("  CLI:      %s\r\n", api.system.cliVersion.get().c_str());
  DBG_PRINTF("  API:      %s\r\n", api.system.apiVersion.get().c_str());
  DBG_PRINTF("  LoRaWAN:  %s\r\n", api.lorawan.ver.get().c_str());
  DBG_PRINTF("  Model:    %s\r\n", api.system.modelId.get().c_str());
  DBG_PRINTF("  Chip ID:  %s\r\n", api.system.chipId.get().c_str());
}

static void log_deveui() {
  uint8_t dev_eui[8] = {0};
  if (api.lorawan.deui.get(dev_eui, sizeof(dev_eui))) {
    DBG_PRINT("[LoRa] DevEUI: ");
    print_hex_compact(dev_eui, sizeof(dev_eui));
    DBG_PRINTLN("");
  } else {
    DBG_PRINTLN("[LoRa] ERROR: Unable to read DevEUI");
  }
}

static void ensure_valid_deveui() {
  uint8_t dev_eui[8] = {0};

  if (!api.lorawan.deui.get(dev_eui, sizeof(dev_eui))) {
    DBG_PRINTLN("[LoRa] ERROR: Unable to read DevEUI for validation");
    return;
  }

  // If chip already has a valid DevEUI (not all zeros), use it
  if (!eui_is_all_zero(dev_eui, sizeof(dev_eui))) {
    DBG_PRINTLN("[LoRa] DevEUI is present in RUI state");
    return;
  }

  // DevEUI is all zeros, generate one from STM32 unique ID
  uint8_t generated_dev_eui[8] = {0};
  read_hardware_uid64(generated_dev_eui);

  DBG_PRINT("[LoRa] DevEUI is 0000000000000000, generating from STM32 UID64: ");
  print_hex_compact(generated_dev_eui, sizeof(generated_dev_eui));
  DBG_PRINTLN("");

  if (!api.lorawan.deui.set(generated_dev_eui, sizeof(generated_dev_eui))) {
    DBG_PRINTLN("[LoRa] ERROR: Failed to set generated DevEUI");
    return;
  }

  DBG_PRINTLN("[LoRa] Generated DevEUI applied");
}

static void log_appeui() {
  uint8_t app_eui[8] = {0};
  if (api.lorawan.appeui.get(app_eui, sizeof(app_eui))) {
    DBG_PRINT("[LoRa] JoinEUI/AppEUI: ");
    print_hex_compact(app_eui, sizeof(app_eui));
    DBG_PRINTLN("");
  } else {
    DBG_PRINTLN("[LoRa] ERROR: Unable to read AppEUI");
  }
}

static void log_lorawan_runtime_config() {
  DBG_PRINTLN("[LoRa] Runtime configuration:");
  DBG_PRINTF("  Net mode:  %s\r\n", network_mode_name(api.lorawan.nwm.get() == 1));
  DBG_PRINTF("  Mode:      %s\r\n", join_mode_name(api.lorawan.njm.get()));
  DBG_PRINTF("  Region:    %s (%ld)\r\n",
             region_name(api.lorawan.band.get()),
             (long)api.lorawan.band.get());
  DBG_PRINTF("  Class:     %s\r\n", class_name(api.lorawan.deviceClass.get()));
  DBG_PRINTF("  ADR:       %s\r\n", api.lorawan.adr.get() ? "ON" : "OFF");
  DBG_PRINTF("  DR:        %u\r\n", api.lorawan.dr.get());
  DBG_PRINTF("  TX Power:  %u dBm\r\n", api.lorawan.txp.get());
  DBG_PRINTF("  Confirmed: %s\r\n", api.lorawan.cfm.get() ? "YES" : "NO");
  DBG_PRINTF("  Retry:     %u\r\n", api.lorawan.rety.get());
  DBG_PRINTF("  Joined:    %s\r\n", api.lorawan.njs.get() ? "YES" : "NO");
}

// ================= SLEEP =================
// RUI sleep in chunks: houdt elke sleep.all() call kort en voorkomt uint32-overloop.
static void sleep_for_ms(uint32_t total_ms) {
  while (total_ms > 0) {
    uint32_t chunk = (total_ms > SLEEP_CHUNK_MS) ? SLEEP_CHUNK_MS : total_ms;
    api.system.sleep.all(chunk);
    total_ms -= chunk;
  }
}

static uint32_t sleep_ms_for_battery_pct(uint8_t bat_pct) {
  if (bat_pct < 15) return SLEEP_6H_MS;
  if (bat_pct < 20) return SLEEP_4H_MS;
  return SLEEP_15MIN_MS;
}

// <10%: "helemaal uit" => ~2 maanden slapen, daarna blijven slapen tot reset
static void battery_cutoff_two_months() {
  for (uint16_t day = 0; day < CUTOFF_DAYS; day++) {
    sleep_for_ms(SLEEP_DAY_MS);
  }
  while (true) {
    sleep_for_ms(SLEEP_DAY_MS);
  }
}

// ================= BME280 FUNCTIES =================
static void bme280_write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t bme280_read_reg(uint8_t reg) {
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0x00;
}

static void bme280_read_coefficients() {
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDR, (uint8_t)6);

  bme_cal.dig_T1 = (uint16_t)(Wire.read() | (Wire.read() << 8));
  bme_cal.dig_T2 = (int16_t)(Wire.read() | (Wire.read() << 8));
  bme_cal.dig_T3 = (int16_t)(Wire.read() | (Wire.read() << 8));

  bme_cal.dig_H1 = bme280_read_reg(0xA1);

  Wire.beginTransmission(BME280_ADDR);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDR, (uint8_t)7);

  bme_cal.dig_H2 = (int16_t)(Wire.read() | (Wire.read() << 8));
  bme_cal.dig_H3 = (uint16_t)Wire.read();
  uint8_t e4 = Wire.read();
  uint8_t e5 = Wire.read();
  uint8_t e6 = Wire.read();
  bme_cal.dig_H4 = (int16_t)((e4 << 4) | (e5 & 0x0F));
  bme_cal.dig_H5 = (int16_t)((e6 << 4) | (e5 >> 4));
  bme_cal.dig_H6 = (int8_t)Wire.read();
}

// An MCU reset does not clock the I2C bus: if the BME280 holds SDA low it keeps
// doing so after the reset. Clock it free and send a STOP before using the bus.
static void i2c_bus_recover() {
  Wire.end();

  pinMode(I2C_SCL_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);

  // Max 9 pulses: a slave can never hold more than 9 bits (8 data + ACK).
  for (uint8_t i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // Manual STOP condition: SDA low, SCL high, then SDA high.
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(5);
}

static bool bme280_init() {
  Wire.begin();
  delay(10);

  uint8_t chipid = bme280_read_reg(BME280_REG_CHIPID);
  if (chipid != BME280_CHIPID) {
    DBG_PRINTF("[BME] Wrong chip ID: 0x%02X (expected 0x60)\r\n", chipid);
    return false;
  }

  DBG_PRINTF("[BME] Chip ID OK: 0x%02X\r\n", chipid);
  bme280_read_coefficients();
  bme280_write_reg(BME280_REG_CTRL_HUM, 0x01);
  bme280_write_reg(BME280_REG_CTRL_MEAS, 0x27);
  bme280_write_reg(BME280_REG_CONFIG, 0xA0);
  delay(100);
  return true;
}

static float bme280_read_temperature() {
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(BME280_REG_TEMP);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDR, (uint8_t)3);

  uint32_t msb = Wire.read();
  uint32_t lsb = Wire.read();
  uint32_t xlsb = Wire.read();
  int32_t adc_T = (int32_t)((msb << 12) | (lsb << 4) | (xlsb >> 4));

  int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_cal.dig_T1 << 1))) *
                  ((int32_t)bme_cal.dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_cal.dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)bme_cal.dig_T1))) >> 12) *
                  ((int32_t)bme_cal.dig_T3)) >> 14;

  bme_cal.t_fine = var1 + var2;
  float temperature = (float)((bme_cal.t_fine * 5 + 128) >> 8);
  return temperature / 100.0f;
}

static float bme280_read_humidity() {
  Wire.beginTransmission(BME280_ADDR);
  Wire.write(BME280_REG_HUM);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDR, (uint8_t)2);

  int32_t adc_H = (int32_t)(((uint32_t)Wire.read() << 8) | Wire.read());

  int32_t v_x1_u32r = (bme_cal.t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme_cal.dig_H4) << 20) -
                 (((int32_t)bme_cal.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
               (((((((v_x1_u32r * ((int32_t)bme_cal.dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)bme_cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                 ((int32_t)2097152)) * ((int32_t)bme_cal.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                            ((int32_t)bme_cal.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

  float humidity = (float)(v_x1_u32r >> 12);
  return humidity / 1024.0f;
}

// Reads T/RH; returns false (and NaN values) if the BME280 is absent or implausible.
static bool bme280_read(float &temp_c, float &humidity_pct) {
  temp_c = NAN;
  humidity_pct = NAN;
  if (!g_bme_ready) return false;

  const float t = bme280_read_temperature();
  const float h = bme280_read_humidity();
  if (!isfinite(t) || t < BME_TEMP_MIN_VALID || t > BME_TEMP_MAX_VALID ||
      !isfinite(h) || h < BME_RH_MIN_VALID   || h > BME_RH_MAX_VALID) {
    return false;
  }
  temp_c = t;
  humidity_pct = h;
  return true;
}

// ================= BATTERY =================
static float read_battery_voltage() {
  uint32_t adc = 0;
  for (int i = 0; i < 8; i++) {
    adc += analogRead(BAT_ADC_PIN);
    delay(1);
  }
  adc /= 8;

  float v_adc = (adc * VREF) / 1023.0f;
  return v_adc * (R_TOP + R_BOT) / R_BOT;
}

static uint8_t battery_percent(float vbat) {
  int pct = map((int)lroundf(vbat * 1000.0f), 3200, 4100, 0, 100);
  return (uint8_t)constrain(pct, 0, 100);
}

// ================= O2 TEMPERATUURCOMPENSATIE =================
// Alphasense O2-A2 datasheet, Fig.1 "Output Temperature Dependence":
// % output relative to the 20degC reference point, vs temperature (degC).
// Identiek aan de MKR1310-firmware.
struct O2TempCompPoint { float temp_c; float rel_output_pct; };
constexpr O2TempCompPoint O2_TEMP_COMP_TABLE[] = {
  { -20.0f,  89.0f },
  { -10.0f,  93.0f },
  {   0.0f,  96.0f },
  {  10.0f,  98.0f },
  {  20.0f, 100.0f },
  {  30.0f, 102.0f },
  {  40.0f, 103.5f },
  {  50.0f, 104.0f },
};
constexpr uint8_t O2_TEMP_COMP_POINTS = sizeof(O2_TEMP_COMP_TABLE) / sizeof(O2_TEMP_COMP_TABLE[0]);

// Linear interpolation over the table, clamped at the table edges.
static float o2_relative_output_pct(float temp_c) {
  if (temp_c <= O2_TEMP_COMP_TABLE[0].temp_c) return O2_TEMP_COMP_TABLE[0].rel_output_pct;

  constexpr uint8_t last = O2_TEMP_COMP_POINTS - 1;
  if (temp_c >= O2_TEMP_COMP_TABLE[last].temp_c) return O2_TEMP_COMP_TABLE[last].rel_output_pct;

  for (uint8_t i = 0; i < last; i++) {
    const float t0 = O2_TEMP_COMP_TABLE[i].temp_c;
    const float t1 = O2_TEMP_COMP_TABLE[i + 1].temp_c;
    if (temp_c >= t0 && temp_c <= t1) {
      const float r0 = O2_TEMP_COMP_TABLE[i].rel_output_pct;
      const float r1 = O2_TEMP_COMP_TABLE[i + 1].rel_output_pct;
      const float frac = (temp_c - t0) / (t1 - t0);
      return r0 + frac * (r1 - r0);
    }
  }
  return 100.0f; // unreachable
}

// Rescales a raw O2 reading (taken at ambient_c, using a reference calibrated at cal_c)
// back to what the sensor would read at the calibration temperature.
static float apply_o2_temp_compensation(float o2_raw, float ambient_c, float cal_c) {
  if (!isfinite(ambient_c) || !isfinite(cal_c)) return o2_raw;
  const float rel_now = o2_relative_output_pct(ambient_c);
  const float rel_cal = o2_relative_output_pct(cal_c);
  if (rel_now <= 0.0f) return o2_raw;
  return o2_raw * (rel_cal / rel_now);
}

// ================= O2 CALIBRATIE IN FLASH =================
static uint32_t cal_checksum(const CalData &d) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&d);
  uint32_t sum = 0;
  for (size_t i = 0; i < sizeof(CalData) - sizeof(uint32_t); i++) sum += p[i];
  return sum;
}

static bool is_valid_o2_cal_v(float v) {
  return isfinite(v) && v >= O2_CAL_V_MIN_VALID && v <= O2_CAL_V_MAX_VALID;
}

static bool is_valid_o2_cal_adc(float adc_counts) {
  return isfinite(adc_counts) && adc_counts >= O2_CAL_ADC_MIN_VALID && adc_counts <= O2_ADC_MAX_VALID;
}

static bool load_calibration(float &out_cal_v, float &out_cal_temp) {
  CalData d = {};
  if (!api.system.flash.get(CAL_FLASH_OFFSET, reinterpret_cast<uint8_t *>(&d), sizeof(d))) return false;
  if (d.magic != CAL_MAGIC) return false;
  if (cal_checksum(d) != d.checksum) return false;
  if (!is_valid_o2_cal_v(d.o2_cal_v)) return false;
  out_cal_v    = d.o2_cal_v;
  out_cal_temp = isfinite(d.cal_temp) ? d.cal_temp : O2_CAL_DEFAULT_TEMP_C;
  return true;
}

static bool save_calibration(float cal_v, float cal_temp_c) {
  if (!is_valid_o2_cal_v(cal_v)) {
    DBG_PRINTF("[CAL] Refusing to save invalid O2 reference: %.5f V\r\n", cal_v);
    return false;
  }

  CalData d = {};
  d.magic    = CAL_MAGIC;
  d.o2_cal_v = cal_v;
  d.cal_temp = cal_temp_c;
  d.checksum = cal_checksum(d);

  if (!api.system.flash.set(CAL_FLASH_OFFSET, reinterpret_cast<uint8_t *>(&d), sizeof(d))) {
    DBG_PRINTLN("[CAL] ERROR: flash write failed");
    return false;
  }

  DBG_PRINTF("[CAL] Saved O2 reference to flash: %.5f V @ %.2f degC\r\n", cal_v, cal_temp_c);
  return true;
}

// ================= O2 METING =================
static float o2_percent_from_adc_counts(float adc_counts, float cal_v) {
  const float v = adc_counts * (O2_ADC_VREF / O2_SENSOR_FULL_SCALE_COUNTS);
  return v * (O2_DEFAULT_CAL_GAS / cal_v);
}

static float read_o2_adc_avg_counts() {
  uint32_t sum = 0;
  constexpr int NUM_SAMPLES = 32;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(O2_ADC_PIN);
    delay(O2_ADC_SAMPLE_DELAY_MS);
  }
  return (float)sum / (float)NUM_SAMPLES;
}

static float read_o2_adc_avg_counts_fast(uint8_t sample_count, uint32_t sample_delay_ms) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < sample_count; i++) {
    sum += analogRead(O2_ADC_PIN);
    if (sample_delay_ms > 0) delay(sample_delay_ms);
  }
  return (float)sum / (float)sample_count;
}

static float read_o2_adc_avg_counts_stabilized() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < O2_STABILIZED_SAMPLES; i++) {
    sum += read_o2_adc_avg_counts();
    delay(O2_STABILIZED_DELAY_MS);
  }
  return sum / (float)O2_STABILIZED_SAMPLES;
}

// A galvanic O2-A2 cell outputs ~0 at 0% O2, exactly like an unplugged cell, so a
// low ADC value alone cannot tell them apart. With the internal pull-up enabled the
// driven OPA333 output keeps PB4 LOW, while a floating input is pulled HIGH. Only
// meaningful for low readings: at normal air the output (~2.2 V) reads HIGH anyway.
static bool is_o2_input_floating() {
  pinMode(O2_ADC_PIN, INPUT_PULLUP);
  delay(O2_OPEN_CIRCUIT_SETTLE_MS);
  const bool floating = digitalRead(O2_ADC_PIN) == HIGH;
  pinMode(O2_ADC_PIN, INPUT);
  return floating;
}

// Shared validation for debug and production readings. Low O2 (down to 0%) is a
// valid measurement; 65535 is only sent for a real sensor fault.
static O2Reading finish_o2_reading(float adc_avg, float adc_min, float adc_max, float ambient_c) {
  O2Reading reading = {NAN, adc_avg, adc_min, adc_max, false, O2_STATUS_NO_VALID_CALIBRATION, false, false};

  if (!is_valid_o2_cal_v(g_o2_cal_v)) return reading;

  if (!isfinite(adc_avg)) {
    reading.status = O2_STATUS_ADC_TOO_LOW;
    return reading;
  }
  if (adc_avg > O2_ADC_MAX_VALID) {
    reading.status = O2_STATUS_ADC_TOO_HIGH;
    return reading;
  }
  if (adc_avg < O2_ADC_LOW_CHECK) {
    reading.open_circuit_tested = true;
    reading.open_circuit = is_o2_input_floating();
    if (reading.open_circuit) {
      reading.status = O2_STATUS_SENSOR_DISCONNECTED;
      return reading;
    }
  }

  const float o2_raw = o2_percent_from_adc_counts(adc_avg, g_o2_cal_v);
  const float o2 = apply_o2_temp_compensation(o2_raw, ambient_c, g_o2_cal_temp);
  reading.o2 = o2;

  if (!isfinite(o2) || o2 < O2_PERCENT_MIN_VALID || o2 > O2_PERCENT_MAX_VALID) {
    reading.status = O2_STATUS_O2_OUT_OF_RANGE;
    return reading;
  }

  reading.valid = true;
  reading.status = O2_STATUS_OK;
  return reading;
}

static O2Reading read_o2_percent_validated(float ambient_c) {
  float adc_min = 0.0f;
  float adc_max = 0.0f;
  float adc_sum = 0.0f;
  for (uint8_t i = 0; i < O2_DEBUG_STABILIZED_READS; i++) {
    const float adc = read_o2_adc_avg_counts_stabilized();
    if (i == 0 || adc < adc_min) adc_min = adc;
    if (i == 0 || adc > adc_max) adc_max = adc;
    adc_sum += adc;
  }
  return finish_o2_reading(adc_sum / (float)O2_DEBUG_STABILIZED_READS, adc_min, adc_max, ambient_c);
}

static O2Reading read_o2_percent_production_validated(float ambient_c) {
  const float adc = read_o2_adc_avg_counts_fast(O2_PRODUCTION_ADC_SAMPLES,
                                                O2_PRODUCTION_ADC_SAMPLE_DELAY_MS);
  return finish_o2_reading(adc, adc, adc, ambient_c);
}

// ================= O2 KALIBRATIE (debug mode) =================
static bool try_compute_stable_calibration_adc(float &adc_avg_out, float &win_min_out, float &win_max_out) {
  float window[O2_CAL_STABILITY_POINTS] = {0.0f};
  uint8_t samples_collected = 0;
  const uint32_t start_ms = millis();

  DBG_PRINTLN("[CAL] Warm-up started");
  DBG_PRINTF("[CAL] Need >= %lus warm-up and a stable ADC window.\r\n",
             (unsigned long)(O2_CAL_MIN_WARMUP_MS / 1000UL));

  while (millis() - start_ms <= O2_CAL_MAX_WARMUP_MS) {
    const float adc_avg = read_o2_adc_avg_counts_stabilized();
    window[samples_collected % O2_CAL_STABILITY_POINTS] = adc_avg;
    samples_collected++;

    DBG_PRINTF("[CAL] t=%lus ADC=%.1f\r\n", (unsigned long)((millis() - start_ms) / 1000UL), adc_avg);

    if (samples_collected >= O2_CAL_STABILITY_POINTS) {
      float win_min = window[0];
      float win_max = window[0];
      float sum = 0.0f;
      for (uint8_t i = 0; i < O2_CAL_STABILITY_POINTS; i++) {
        const float value = window[i];
        if (value < win_min) win_min = value;
        if (value > win_max) win_max = value;
        sum += value;
      }

      const float delta = win_max - win_min;
      DBG_PRINTF("[CAL] Stability window: min=%.1f max=%.1f delta=%.1f\r\n", win_min, win_max, delta);

      if (millis() - start_ms < O2_CAL_MIN_WARMUP_MS) {
        DBG_PRINTLN("[CAL] Warm-up time not reached yet.");
      } else if (!is_valid_o2_cal_adc(adc_avg)) {
        DBG_PRINTLN(adc_avg < O2_CAL_ADC_MIN_VALID
                      ? "[CAL] ADC too low -> calibration blocked."
                      : "[CAL] ADC too high / near saturation -> calibration blocked.");
      } else if (delta > O2_CAL_STABILITY_DELTA_MAX) {
        DBG_PRINTLN("[CAL] ADC window not stable yet.");
      } else {
        adc_avg_out = sum / (float)O2_CAL_STABILITY_POINTS;
        win_min_out = win_min;
        win_max_out = win_max;
        DBG_PRINTLN("[CAL] ADC stable, starting calibration");
        return true;
      }
    }

    if (millis() - start_ms + O2_CAL_SAMPLE_INTERVAL_MS > O2_CAL_MAX_WARMUP_MS) break;
    delay(O2_CAL_SAMPLE_INTERVAL_MS);
  }

  DBG_PRINTLN("[CAL] Sensor not stable after timeout -> not saved");
  return false;
}

static void run_o2_auto_calibration(float cal_gas_o2) {
  float adc_avg = 0.0f;
  float win_min = 0.0f;
  float win_max = 0.0f;
  if (!try_compute_stable_calibration_adc(adc_avg, win_min, win_max)) return;

  if (!is_valid_o2_cal_adc(adc_avg)) {
    DBG_PRINTLN("[CAL] ADC outside valid range -> not saved");
    return;
  }

  // Spanning bij kalibratiegas, teruggerekend naar de spanning bij 20.9% O2
  const float avg_v = adc_avg * (O2_ADC_VREF / O2_SENSOR_FULL_SCALE_COUNTS);
  const float new_cal_v = avg_v * (O2_DEFAULT_CAL_GAS / cal_gas_o2);

  DBG_PRINTLN("");
  DBG_PRINTLN("=== O2 AUTO CAL ===");
  DBG_PRINTF("[CAL] Gas O2%%: %.2f\r\n", cal_gas_o2);
  DBG_PRINTF("[CAL] ADC avg: %.1f (%.4f V)\r\n", adc_avg, avg_v);
  DBG_PRINTF("[CAL] Stability min/max: %.1f / %.1f\r\n", win_min, win_max);
  DBG_PRINTF("[CAL] New O2 reference: %.5f V @ 20.9%%\r\n", new_cal_v);

  if (!is_valid_o2_cal_v(new_cal_v)) {
    DBG_PRINTF("[CAL] New O2 reference outside valid range [%.2f, %.2f] -> not saved\r\n",
               O2_CAL_V_MIN_VALID, O2_CAL_V_MAX_VALID);
    return;
  }

  float cal_temp_c = NAN;
  float cal_rh = NAN;
  (void)bme280_read(cal_temp_c, cal_rh);

  g_o2_cal_v    = new_cal_v;
  g_o2_cal_temp = isfinite(cal_temp_c) ? cal_temp_c : O2_CAL_DEFAULT_TEMP_C;
  if (!save_calibration(g_o2_cal_v, g_o2_cal_temp)) return;

  DBG_PRINTF("[CAL] Calibration temp: %.2f degC\r\n", g_o2_cal_temp);
  const O2Reading check = read_o2_percent_validated(g_o2_cal_temp);
  if (check.valid) {
    DBG_PRINTF("[CAL] Check O2 now: %.2f %%\r\n", check.o2);
  } else {
    DBG_PRINTF("[CAL] Check O2 now: invalid (%s)\r\n", o2_status_name(check.status));
  }
  DBG_PRINTLN("===============");
  DBG_PRINTLN("");
}

static bool read_serial_line_with_timeout(String &out, uint32_t timeout_ms) {
  out = "";
  while (Serial.available() > 0) {
    (void)Serial.read();
  }

  uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        return true;
      }
      out += c;
    }
    delay(5);
  }
  return false;
}

static void run_o2_calibration_from_boot_prompt() {
  float cal_gas_o2 = O2_DEFAULT_CAL_GAS;

  DBG_PRINTLN("[CAL] Calibration mode active.");
  DBG_PRINTF("[CAL] Default gas = %.2f%% (fresh air).\r\n", O2_DEFAULT_CAL_GAS);
  DBG_PRINTF("[CAL] Enter other O2%% within %lus + Enter, or wait to use %.2f.\r\n",
             (unsigned long)(CAL_GAS_INPUT_TIMEOUT_MS / 1000UL), O2_DEFAULT_CAL_GAS);
  DBG_PRINT("> ");

  String line;
  if (read_serial_line_with_timeout(line, CAL_GAS_INPUT_TIMEOUT_MS)) {
    line.trim();
    if (line.length() > 0) {
      line.replace(',', '.');
      float v = line.toFloat();
      if (v > O2_PERCENT_MIN_VALID && v < O2_PERCENT_CAL_INPUT_MAX) {
        cal_gas_o2 = v;
        DBG_PRINTF("[CAL] Using entered gas: %.2f\r\n", cal_gas_o2);
      } else {
        DBG_PRINTLN("[CAL] Invalid input -> using default gas.");
      }
    } else {
      DBG_PRINTLN("[CAL] Empty -> using default gas.");
    }
  } else {
    DBG_PRINTLN("");
    DBG_PRINTLN("[CAL] No input -> using default gas.");
  }

  DBG_PRINTLN("[CAL] Starting warm-up and stability check...");
  run_o2_auto_calibration(cal_gas_o2);
}

// ================= PAYLOAD =================
static PayloadV1 make_payload(const O2Reading &o2_reading, float temp_c, float humidity_pct, uint8_t bat_pct) {
  PayloadV1 payload = {};
  payload.msgType = 0x01;

  // Keep the 8-byte payload unchanged by reserving 65535 as an invalid O2 marker.
  // Valid high-air readings are normalized to 20.9% for a clean overview across sensors.
  if (o2_reading.valid) {
    float o2c = o2_reading.o2;
    if (o2c < O2_PERCENT_MIN_VALID) o2c = O2_PERCENT_MIN_VALID;
    if (o2c > O2_PERCENT_MAX_VALID) o2c = O2_PERCENT_MAX_VALID;
    if (o2c > O2_PERCENT_AIR_DISPLAY_CAP) o2c = O2_DEFAULT_CAL_GAS;
    payload.o2_x10 = (uint16_t)lroundf(o2c * 10.0f);
  } else {
    payload.o2_x10 = O2_PAYLOAD_INVALID_MARKER;
  }

  payload.t_x10   = isfinite(temp_c) ? (int16_t)lroundf(temp_c * 10.0f) : T_PAYLOAD_INVALID_MARKER;
  payload.h_x10   = isfinite(humidity_pct) ? (uint16_t)lroundf(humidity_pct * 10.0f) : H_PAYLOAD_INVALID_MARKER;
  payload.bat_pct = (uint8_t)constrain((int)bat_pct, 0, 100);
  return payload;
}

static void log_payload_preview(const PayloadV1 &payload) {
  DBG_PRINTLN("[DEBUG] Payload preview (NOT sent):");
  DBG_PRINTF("  msgType: 0x%02X\r\n", payload.msgType);
  DBG_PRINTF("  o2_x10:  %u\r\n", payload.o2_x10);
  DBG_PRINTF("  t_x10:   %d\r\n", payload.t_x10);
  DBG_PRINTF("  h_x10:   %u\r\n", payload.h_x10);
  DBG_PRINTF("  bat:     %u\r\n", payload.bat_pct);
  DBG_PRINT("  HEX:     ");
  print_hex_compact((const uint8_t *)&payload, sizeof(payload));
  DBG_PRINTLN("");
}

static void run_debug_status() {
  float temp_c = NAN;
  float humidity_pct = NAN;
  (void)bme280_read(temp_c, humidity_pct);
  const O2Reading o2 = read_o2_percent_validated(temp_c);
  const float vbat = read_battery_voltage();
  const uint8_t bat_pct = battery_percent(vbat);
  const PayloadV1 preview = make_payload(o2, temp_c, humidity_pct, bat_pct);

  DBG_PRINTLN("");
  DBG_PRINTLN("----- DEBUG STATUS -----");
  DBG_PRINTF("[DEBUG] Temp: %.2f C\r\n", temp_c);
  DBG_PRINTF("[DEBUG] RH:   %.2f %%\r\n", humidity_pct);
  DBG_PRINTF("[DEBUG] Battery: %.3f V (%u%%)\r\n", vbat, bat_pct);
  DBG_PRINTF("[DEBUG] O2 status: %s\r\n", o2_status_name(o2.status));
  DBG_PRINTF("[DEBUG] O2 ADC avg: %.1f\r\n", o2.adc);
  DBG_PRINTF("[DEBUG] O2 ADC min: %.1f\r\n", o2.adc_min);
  DBG_PRINTF("[DEBUG] O2 ADC max: %.1f\r\n", o2.adc_max);
  if (!o2.open_circuit_tested) {
    DBG_PRINTLN("[DEBUG] O2 open-circuit test: not needed");
  } else {
    DBG_PRINTF("[DEBUG] O2 open-circuit test: %s\r\n", o2.open_circuit ? "FLOATING" : "connected");
  }
  if (o2.valid) {
    DBG_PRINTF("[DEBUG] O2 value: %.2f %%\r\n", o2.o2);
    DBG_PRINTF("[DEBUG] O2 tx : %.2f %%\r\n",
               o2.o2 > O2_PERCENT_AIR_DISPLAY_CAP ? O2_DEFAULT_CAL_GAS : o2.o2);
  } else {
    DBG_PRINTLN("[DEBUG] O2 value: invalid");
    DBG_PRINTLN("[DEBUG] O2 tx : invalid");
  }
  DBG_PRINTF("[DEBUG] O2 reference: %.5f V @ 20.9%%\r\n", g_o2_cal_v);
  DBG_PRINTF("[DEBUG] O2 cal temp: %.2f degC\r\n", g_o2_cal_temp);
  DBG_PRINTF("[DEBUG] O2 temp comp factor: %.4f\r\n",
             o2_relative_output_pct(g_o2_cal_temp) / o2_relative_output_pct(temp_c));
  log_payload_preview(preview);
  DBG_PRINTLN("------------------------");
}

// ================= LORAWAN =================
static bool ensure_lorawan_mode() {
  if (api.lorawan.nwm.get() == 1) {
    DBG_PRINTLN("[LoRa] Device already in LoRaWAN mode");
    return true;
  }

  DBG_PRINTLN("[LoRa] Device is in P2P mode, switching to LoRaWAN and rebooting...");
  if (!api.lorawan.nwm.set()) {
    DBG_PRINTLN("[LoRa] ERROR: failed to switch device to LoRaWAN mode");
    return false;
  }

  DBG_PRINTLN("[LoRa] Mode changed, rebooting now");
  delay(250);
  api.system.reboot();
  while (true) {
    delay(1000);
  }
}

static bool configure_lorawan() {
  if (!validate_and_log_secrets()) {
    return false;
  }

  if (!ensure_lorawan_mode()) {
    return false;
  }

  uint8_t app_eui[8] = {0};
  uint8_t app_key[16] = {0};

  if (!hex_to_bytes(SECRET_APP_EUI, app_eui, sizeof(app_eui))) {
    DBG_PRINTLN("[LoRa] ERROR: could not parse SECRET_APP_EUI");
    return false;
  }
  if (!hex_to_bytes(SECRET_APP_KEY, app_key, sizeof(app_key))) {
    DBG_PRINTLN("[LoRa] ERROR: could not parse SECRET_APP_KEY");
    return false;
  }

  ensure_valid_deveui();
  api.lorawan.registerSendCallback(sendCallback);
  api.lorawan.registerJoinCallback(joinCallback);

  if (!api.lorawan.band.set(RAK_REGION_EU868)) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set region EU868");
    return false;
  }
  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A)) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set Class A");
    return false;
  }
  if (!api.lorawan.njm.set(RAK_LORA_OTAA)) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set OTAA mode");
    return false;
  }
  if (!api.lorawan.appeui.set(app_eui, sizeof(app_eui))) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set JoinEUI/AppEUI");
    return false;
  }
  if (!api.lorawan.appkey.set(app_key, sizeof(app_key))) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set AppKey");
    return false;
  }
  if (!api.lorawan.adr.set(true)) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set ADR");
    return false;
  }
  if (!api.lorawan.cfm.set(LORA_CONFIRMED_UPLINK ? 1 : 0)) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set confirmed mode");
    return false;
  }
  if (!api.lorawan.rety.set(LORA_SEND_RETRY)) {
    DBG_PRINTLN("[LoRa] ERROR: failed to set retry count");
    return false;
  }

  DBG_PRINTLN("[LoRa] Join identity after configuration:");
  log_deveui();
  log_appeui();
  log_lorawan_runtime_config();
  return true;
}

static bool run_join_attempts() {
  for (uint8_t attempt = 1; attempt <= JOIN_MAX_ATTEMPTS; attempt++) {
    DBG_PRINTF("[JOIN] Attempt %u/%u: starting OTAA join...\r\n",
               attempt, JOIN_MAX_ATTEMPTS);

    g_join_event_seen = false;
    g_last_join_status = INT32_MIN;

    if (!api.lorawan.join()) {
      DBG_PRINTLN("[JOIN] Start failed: join request was not accepted by the stack");
      if (attempt < JOIN_MAX_ATTEMPTS) {
        delay(JOIN_RETRY_DELAY_MS);
      }
      continue;
    }

    DBG_PRINTF("[JOIN] Start accepted, waiting up to %lu seconds for join accept...\r\n",
               JOIN_ATTEMPT_TIMEOUT_MS / 1000UL);

    uint32_t started_at = millis();
    uint32_t last_progress_log = started_at;

    while ((millis() - started_at) < JOIN_ATTEMPT_TIMEOUT_MS) {
      if (api.lorawan.njs.get()) {
        DBG_PRINTLN("[JOIN] Join complete: network session is active");
        g_join_announced = true;
        return true;
      }

      if ((millis() - last_progress_log) >= 10000UL) {
        DBG_PRINTLN("[JOIN] Waiting for accept...");
        last_progress_log = millis();
      }
      delay(JOIN_POLL_INTERVAL_MS);
    }

    if (api.lorawan.njs.get()) {
      DBG_PRINTLN("[JOIN] Join complete after final state check");
      g_join_announced = true;
      return true;
    }

    if (g_join_event_seen) {
      DBG_PRINTF("[JOIN] Attempt %u ended without session, callback=%s\r\n",
                 attempt, loramac_status_name(g_last_join_status));
    } else {
      DBG_PRINTF("[JOIN] Attempt %u timed out: join started but no accept was observed\r\n",
                 attempt);
    }

    if (attempt < JOIN_MAX_ATTEMPTS) {
      DBG_PRINTF("[JOIN] Retrying in %lu seconds...\r\n",
                 JOIN_RETRY_DELAY_MS / 1000UL);
      delay(JOIN_RETRY_DELAY_MS);
    }
  }

  return false;
}

static bool ensure_joined() {
  if (api.lorawan.njs.get()) {
    if (!g_join_announced) {
      DBG_PRINTLN("[JOIN] Network already joined");
      g_join_announced = true;
    }
    return true;
  }

  if (run_join_attempts()) {
    return true;
  }

  DBG_PRINTLN("[JOIN] FAILED: either credentials/config are wrong, device state was invalid, or RF/network accept is missing");
  return false;
}

static bool wait_for_send_result(uint32_t timeout_ms) {
  uint32_t started_at = millis();
  while (!g_send_event_seen && (millis() - started_at) < timeout_ms) {
    delay(50);
  }

  if (!g_send_event_seen) {
    DBG_PRINTLN("[SEND] Timeout while waiting for send callback");
    return false;
  }

  if (g_last_send_status == RAK_LORAMAC_STATUS_OK) {
    DBG_PRINTLN("[SEND] Uplink finished successfully");
    return true;
  }

  DBG_PRINTF("[SEND] Uplink failed with status %s\r\n",
             loramac_status_name(g_last_send_status));
  return false;
}

static bool send_measurement_payload(uint8_t bat_pct) {
  // Without BME280: O2 + battery still sent, T/RH as invalid markers, no live temp comp
  float temp_c = NAN;
  float humidity_pct = NAN;
  (void)bme280_read(temp_c, humidity_pct);
  const O2Reading o2 = read_o2_percent_production_validated(temp_c);
  PayloadV1 payload = make_payload(o2, temp_c, humidity_pct, bat_pct);

  g_send_event_seen = false;
  g_last_send_status = INT32_MIN;

  if (!api.lorawan.send(sizeof(payload),
                        (uint8_t *)&payload,
                        LORA_FPORT,
                        LORA_CONFIRMED_UPLINK,
                        LORA_SEND_RETRY)) {
    return false;
  }

  return wait_for_send_result(SEND_RESULT_TIMEOUT_MS);
}

// ================= SETUP =================
void setup() {
  pinMode(TPL_DONE_PIN, OUTPUT);
  digitalWrite(TPL_DONE_PIN, LOW);

  // ---- Load stored calibration early ----
  float stored_cal_v = 0.0f;
  float stored_cal_temp = O2_CAL_DEFAULT_TEMP_C;
  const bool has_stored = load_calibration(stored_cal_v, stored_cal_temp);
  if (has_stored) {
    g_o2_cal_v    = stored_cal_v;
    g_o2_cal_temp = stored_cal_temp;
  }

  // ---- Determine mode ONLY by PA4 low at boot (within window) ----
  pinMode(SERVICE_MODE_PIN, INPUT_PULLUP);
  uint32_t trigger_start = millis();
  while ((millis() - trigger_start) < DEBUG_BOOT_WINDOW_MS) {
    if (digitalRead(SERVICE_MODE_PIN) == LOW) {
      g_debug_mode = true;
      break;
    }
    delay(5);
  }

  if (g_debug_mode) {
    Serial.begin(115200, RAK_AT_MODE);
    delay(1200);

    DBG_PRINTLN("");
    DBG_PRINTLN("=== System Boot (RAK3172) ===");
    DBG_PRINTLN("[BOOT] Mode: DEBUG+CAL (PA4 LOW)");
    log_rui_versions();
    DBG_PRINTF("[BOOT] O2 reference default: %.5f V\r\n", O2_CAL_V_DEFAULT);
    if (has_stored) {
      DBG_PRINTF("[BOOT] Loaded O2 reference from flash: %.5f V\r\n", g_o2_cal_v);
      DBG_PRINTF("[BOOT] Loaded calibration temp: %.2f degC\r\n", g_o2_cal_temp);
    } else {
      DBG_PRINTLN("[BOOT] No valid flash calibration found -> using default.");
    }
    DBG_PRINTF("[BOOT] PayloadV1 size: %u\r\n", (unsigned)sizeof(PayloadV1));
  }

  // UART1 (PB6/PB7) is unused on the v4 board; disable it to save power
  Serial1.end();

  analogReadResolution(10);
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(O2_ADC_PIN, INPUT);

  // ---- BME280 init (before calibration: needed for calibration temp) ----
  i2c_bus_recover();
  g_bme_ready = bme280_init();
  if (!g_bme_ready) {
    delay(100);
    i2c_bus_recover();
    g_bme_ready = bme280_init();
  }
  if (g_bme_ready) {
    DBG_PRINTLN("[BME] OK");
  } else {
    DBG_PRINTLN("[BME] Sensor not found!");
  }

  // ---- Calibration when PA4 low; daarna blijven we in debug mode zonder uplinks ----
  if (g_debug_mode) {
    // Self-test: the pull-up probe must not break later analogRead() calls
    const float adc_before = read_o2_adc_avg_counts();
    const bool floating = is_o2_input_floating();
    const float adc_after = read_o2_adc_avg_counts();
    DBG_PRINTF("[O2] Open-circuit probe self-test: ADC %.1f -> %.1f, input %s\r\n",
               adc_before, adc_after, floating ? "HIGH" : "LOW");

    run_o2_calibration_from_boot_prompt();
    DBG_PRINTF("[MODE] Debug mode: no join, no uplink. Status every 30s. Auto-switch to production after %lu min.\r\n",
               (unsigned long)(DEBUG_MODE_MAX_MS / 60000UL));
    return;
  }

  g_lora_ready = configure_lorawan();
}

// ================= LOOP =================
void loop() {
  // ==========================================================
  // MODE 1: DEBUG+CAL -> Serial prints + payload preview, NO uplink
  // ==========================================================
  if (g_debug_mode && millis() >= DEBUG_MODE_MAX_MS) {
    DBG_PRINTLN("[MODE] Debug time limit reached -> switching to production (PA4 jumper ignored)");
    DBG_PRINTLN("[MODE] Serial logging stops now.");
    delay(100);
    g_debug_mode = false;
    g_lora_ready = configure_lorawan();
  }

  if (g_debug_mode) {
    static uint32_t last_preview_at = 0;
    static bool first_preview = true;
    if (first_preview || (millis() - last_preview_at) >= DEBUG_PAYLOAD_PREVIEW_MS) {
      first_preview = false;
      last_preview_at = millis();
      run_debug_status();
    }
    delay(100);
    return;
  }

  // ==========================================================
  // MODE 2: PRODUCTIE -> uplink, daarna batterij-afhankelijke RUI sleep
  // ==========================================================
  const float vbat = read_battery_voltage();
  const uint8_t bat_pct = battery_percent(vbat);
  if (bat_pct < BAT_CUTOFF_PCT) {
    battery_cutoff_two_months(); // keert niet terug
  }

  if (g_lora_ready && ensure_joined()) {
    (void)send_measurement_payload(bat_pct);
  }

  sleep_for_ms(sleep_ms_for_battery_pct(bat_pct));
}
