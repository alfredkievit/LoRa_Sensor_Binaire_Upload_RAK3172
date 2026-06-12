/*
  RAK3172 RUI3/RUI4 - O2 + BME280 sensor
  VS Code / Arduino workflow, join-first diagnostics

  Target board package detected locally:
    rak_rui stm32 4.2.4
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "arduino_secrets.h"

// ================= CONFIG =================
constexpr uint32_t SEND_INTERVAL_MS          = 20UL * 60UL * 1000UL;
constexpr uint32_t JOIN_ATTEMPT_TIMEOUT_MS   = 90UL * 1000UL;
constexpr uint32_t JOIN_RETRY_DELAY_MS       = 5UL * 1000UL;
constexpr uint32_t JOIN_POLL_INTERVAL_MS     = 2UL * 1000UL;
constexpr uint8_t  JOIN_MAX_ATTEMPTS         = 3;
constexpr uint8_t  LORA_FPORT                = 1;
constexpr bool     LORA_CONFIRMED_UPLINK     = false;
constexpr uint8_t  LORA_SEND_RETRY           = 1;
constexpr bool     JOIN_DIAGNOSTICS_ONLY     = false;
constexpr bool     ENABLE_LOW_POWER_SLEEP    = false;

// ================= PINMAP =================
#define I2C_SDA_PIN         PB7
#define I2C_SCL_PIN         PB6
#define BAT_ADC_PIN         PB3
#define O2_ADC_PIN          PB2
#define PWR_ENABLE_PIN      PA8

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
constexpr float R_TOP = 470000.0f;
constexpr float R_BOT = 1500000.0f;
constexpr float VREF  = 3.3f;

// ================= O2 =================
constexpr float O2_VREF = 3.1714f;

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
  uint8_t  msgType;
  uint16_t o2_x10;
  int16_t  t_x10;
  uint16_t h_x10;
  uint8_t  bat_pct;
};

static_assert(sizeof(PayloadV1) == 8, "PayloadV1 must remain 8 bytes");

// ================= RUNTIME STATE =================
static bool     g_bme_ready = false;
static bool     g_join_announced = false;
static volatile bool    g_join_event_seen = false;
static volatile int32_t g_last_join_status = INT32_MIN;
static volatile bool    g_send_event_seen = false;
static volatile int32_t g_last_send_status = INT32_MIN;

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

static bool bytes_equal(const uint8_t *lhs, const uint8_t *rhs, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (lhs[i] != rhs[i]) return false;
  }
  return true;
}

// ================= CALLBACKS =================
void joinCallback(int32_t status) {
  g_join_event_seen = true;
  g_last_join_status = status;
  Serial.printf("[JOIN] Callback status=%ld (%s)\r\n",
                (long)status, loramac_status_name(status));
}

void sendCallback(int32_t status) {
  g_send_event_seen = true;
  g_last_send_status = status;
  Serial.printf("[SEND] Callback status=%ld (%s)\r\n",
                (long)status, loramac_status_name(status));
}

// ================= HELPERS =================
static void print_hex_compact(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X", data[i]);
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

  Serial.println("[SECRETS] Validating JoinEUI/AppKey...");

  if (!is_hex_string(SECRET_APP_EUI, 16)) {
    Serial.println("[SECRETS] ERROR: SECRET_APP_EUI must be exactly 16 hex chars");
    ok = false;
  } else {
    Serial.println("[SECRETS] JoinEUI/AppEUI format OK");
  }

  if (!is_hex_string(SECRET_APP_KEY, 32)) {
    Serial.println("[SECRETS] ERROR: SECRET_APP_KEY must be exactly 32 hex chars");
    ok = false;
  } else {
    Serial.println("[SECRETS] AppKey format OK");
  }

  return ok;
}

static void log_rui_versions() {
  Serial.println("[RUI] Runtime versions:");
  Serial.printf("  Firmware: %s\r\n", api.system.firmwareVersion.get().c_str());
  Serial.printf("  CLI:      %s\r\n", api.system.cliVersion.get().c_str());
  Serial.printf("  API:      %s\r\n", api.system.apiVersion.get().c_str());
  Serial.printf("  LoRaWAN:  %s\r\n", api.lorawan.ver.get().c_str());
  Serial.printf("  Model:    %s\r\n", api.system.modelId.get().c_str());
  Serial.printf("  Chip ID:  %s\r\n", api.system.chipId.get().c_str());
}

static void log_deveui() {
  uint8_t dev_eui[8] = {0};
  if (api.lorawan.deui.get(dev_eui, sizeof(dev_eui))) {
    Serial.print("[LoRa] DevEUI: ");
    print_hex_compact(dev_eui, sizeof(dev_eui));
    Serial.println();
  } else {
    Serial.println("[LoRa] ERROR: Unable to read DevEUI");
  }
}

static void ensure_valid_deveui() {
  uint8_t dev_eui[8] = {0};
  if (!api.lorawan.deui.get(dev_eui, sizeof(dev_eui))) {
    Serial.println("[LoRa] ERROR: Unable to read DevEUI for validation");
    return;
  }

  if (!eui_is_all_zero(dev_eui, sizeof(dev_eui))) {
    Serial.println("[LoRa] DevEUI is already present in RUI state");
    return;
  }

  uint8_t generated_dev_eui[8] = {0};
  read_hardware_uid64(generated_dev_eui);

  Serial.print("[LoRa] DevEUI is 0000000000000000, generating from STM32 UID64: ");
  print_hex_compact(generated_dev_eui, sizeof(generated_dev_eui));
  Serial.println();

  if (!api.lorawan.deui.set(generated_dev_eui, sizeof(generated_dev_eui))) {
    Serial.println("[LoRa] ERROR: Failed to set generated DevEUI");
    return;
  }

  Serial.println("[LoRa] Generated DevEUI applied");
}

static void log_appeui() {
  uint8_t app_eui[8] = {0};
  if (api.lorawan.appeui.get(app_eui, sizeof(app_eui))) {
    Serial.print("[LoRa] JoinEUI/AppEUI: ");
    print_hex_compact(app_eui, sizeof(app_eui));
    Serial.println();
  } else {
    Serial.println("[LoRa] ERROR: Unable to read AppEUI");
  }
}

static void log_lorawan_runtime_config() {
  Serial.println("[LoRa] Runtime configuration:");
  Serial.printf("  Net mode:  %s\r\n", network_mode_name(api.lorawan.nwm.get() == 1));
  Serial.printf("  Mode:      %s\r\n", join_mode_name(api.lorawan.njm.get()));
  Serial.printf("  Region:    %s (%ld)\r\n",
                region_name(api.lorawan.band.get()),
                (long)api.lorawan.band.get());
  Serial.printf("  Class:     %s\r\n", class_name(api.lorawan.deviceClass.get()));
  Serial.printf("  ADR:       %s\r\n", api.lorawan.adr.get() ? "ON" : "OFF");
  Serial.printf("  DR:        %u\r\n", api.lorawan.dr.get());
  Serial.printf("  TX Power:  %u dBm\r\n", api.lorawan.txp.get());
  Serial.printf("  Confirmed: %s\r\n", api.lorawan.cfm.get() ? "YES" : "NO");
  Serial.printf("  Retry:     %u\r\n", api.lorawan.rety.get());
  Serial.printf("  Joined:    %s\r\n", api.lorawan.njs.get() ? "YES" : "NO");
}

static bool allow_failed_set_if_value_matches(const char *label,
                                              bool set_ok,
                                              uint32_t current_value,
                                              uint32_t expected_value) {
  if (set_ok) return true;
  if (current_value == expected_value) {
    Serial.printf("[LoRa] %s already matches desired value, continuing\r\n", label);
    return true;
  }

  Serial.printf("[LoRa] ERROR: failed to set %s (current=%lu expected=%lu)\r\n",
                label,
                (unsigned long)current_value,
                (unsigned long)expected_value);
  return false;
}

static bool allow_failed_set_if_bytes_match(const char *label,
                                            bool set_ok,
                                            bool get_ok,
                                            const uint8_t *current_value,
                                            const uint8_t *expected_value,
                                            size_t len) {
  if (set_ok) return true;
  if (get_ok && bytes_equal(current_value, expected_value, len)) {
    Serial.printf("[LoRa] %s already matches desired value, continuing\r\n", label);
    return true;
  }

  Serial.printf("[LoRa] ERROR: failed to set %s and readback does not match\r\n", label);
  return false;
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

static bool bme280_init() {
  Wire.begin();
  delay(10);

  uint8_t chipid = bme280_read_reg(BME280_REG_CHIPID);
  if (chipid != BME280_CHIPID) {
    Serial.printf("[BME] Wrong chip ID: 0x%02X (expected 0x60)\r\n", chipid);
    return false;
  }

  Serial.printf("[BME] Chip ID OK: 0x%02X\r\n", chipid);
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

// ================= SENSOR FUNCTIES =================
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

static float read_o2_voltage() {
  uint32_t adc = 0;
  for (int i = 0; i < 32; i++) {
    adc += analogRead(O2_ADC_PIN);
    delay(1);
  }
  adc /= 32;

  return (adc * O2_VREF) / 1023.0f;
}

static float read_o2_percent_raw() {
  // ME2-O2-20 + OPA333: Vout ~1.0V at 20.9% O2.
  // Formula: O2% = Vout * (20.9 / Vout_at_20.9%)
  // Measured ~1.04V at 20.9%, so reference = 1.0V
  return read_o2_voltage() * 0.21f / 1.0f * 100.0f;
}

// ================= PAYLOAD =================
static PayloadV1 make_payload(float o2_raw, float temp_c, float humidity_pct, uint8_t bat_pct) {
  PayloadV1 payload = {};

  float o2_tx = o2_raw;
  if (!isfinite(o2_tx)) o2_tx = 0.0f;
  if (o2_tx < 0.0f) o2_tx = 0.0f;
  if (o2_tx > 20.0f) o2_tx = 20.9f;

  if (!isfinite(temp_c)) temp_c = 0.0f;
  if (!isfinite(humidity_pct)) humidity_pct = 0.0f;

  payload.msgType = 0x01;
  payload.o2_x10  = (uint16_t)lroundf(o2_tx * 10.0f);
  payload.t_x10   = (int16_t)lroundf(temp_c * 10.0f);
  payload.h_x10   = (uint16_t)lroundf(humidity_pct * 10.0f);
  payload.bat_pct = (uint8_t)constrain((int)bat_pct, 0, 100);

  return payload;
}

static void log_payload_preview(const PayloadV1 &payload,
                                float o2_raw,
                                float temp_c,
                                float humidity_pct,
                                float vbat) {
  Serial.println("[PAYLOAD] Preview:");
  Serial.printf("  Raw O2:       %.2f %%\r\n", o2_raw);
  Serial.printf("  Temperature:  %.2f C\r\n", temp_c);
  Serial.printf("  Humidity:     %.2f %%RH\r\n", humidity_pct);
  Serial.printf("  Battery:      %.3f V (%u%%)\r\n", vbat, payload.bat_pct);
  Serial.printf("  msgType:      0x%02X\r\n", payload.msgType);
  Serial.printf("  o2_x10:       %u\r\n", payload.o2_x10);
  Serial.printf("  t_x10:        %d\r\n", payload.t_x10);
  Serial.printf("  h_x10:        %u\r\n", payload.h_x10);
  Serial.print("  Bytes:        ");
  print_hex_compact((const uint8_t *)&payload, sizeof(payload));
  Serial.println();
}

// ================= LORAWAN =================
static bool ensure_lorawan_mode() {
  if (api.lorawan.nwm.get() == 1) {
    Serial.println("[LoRa] Device already in LoRaWAN mode");
    return true;
  }

  Serial.println("[LoRa] Device is in P2P mode, switching to LoRaWAN and rebooting...");
  if (!api.lorawan.nwm.set()) {
    Serial.println("[LoRa] ERROR: failed to switch device to LoRaWAN mode");
    return false;
  }

  Serial.println("[LoRa] Mode changed, rebooting now");
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
    Serial.println("[LoRa] ERROR: could not parse SECRET_APP_EUI");
    return false;
  }
  if (!hex_to_bytes(SECRET_APP_KEY, app_key, sizeof(app_key))) {
    Serial.println("[LoRa] ERROR: could not parse SECRET_APP_KEY");
    return false;
  }

  ensure_valid_deveui();
  api.lorawan.registerSendCallback(sendCallback);
  api.lorawan.registerJoinCallback(joinCallback);

  if (!api.lorawan.band.set(RAK_REGION_EU868)) {
    Serial.println("[LoRa] ERROR: failed to set region EU868");
    return false;
  }
  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A)) {
    Serial.println("[LoRa] ERROR: failed to set Class A");
    return false;
  }
  if (!api.lorawan.njm.set(RAK_LORA_OTAA)) {
    Serial.println("[LoRa] ERROR: failed to set OTAA mode");
    return false;
  }
  if (!api.lorawan.appeui.set(app_eui, sizeof(app_eui))) {
    Serial.println("[LoRa] ERROR: failed to set JoinEUI/AppEUI");
    return false;
  }
  if (!api.lorawan.appkey.set(app_key, sizeof(app_key))) {
    Serial.println("[LoRa] ERROR: failed to set AppKey");
    return false;
  }
  if (!api.lorawan.adr.set(true)) {
    Serial.println("[LoRa] ERROR: failed to set ADR");
    return false;
  }
  if (!api.lorawan.cfm.set(LORA_CONFIRMED_UPLINK ? 1 : 0)) {
    Serial.println("[LoRa] ERROR: failed to set confirmed mode");
    return false;
  }
  if (!api.lorawan.rety.set(LORA_SEND_RETRY)) {
    Serial.println("[LoRa] ERROR: failed to set retry count");
    return false;
  }

  Serial.println("[LoRa] Join identity after configuration:");
  log_deveui();
  log_appeui();
  log_lorawan_runtime_config();
  return true;
}
static bool run_join_attempts() {
  for (uint8_t attempt = 1; attempt <= JOIN_MAX_ATTEMPTS; attempt++) {
    Serial.printf("[JOIN] Attempt %u/%u: starting OTAA join...\r\n",
                  attempt, JOIN_MAX_ATTEMPTS);

    g_join_event_seen = false;
    g_last_join_status = INT32_MIN;

    if (!api.lorawan.join()) {
      Serial.println("[JOIN] Start failed: join request was not accepted by the stack");
      if (attempt < JOIN_MAX_ATTEMPTS) {
        delay(JOIN_RETRY_DELAY_MS);
      }
      continue;
    }

    Serial.printf("[JOIN] Start accepted, waiting up to %lu seconds for join accept...\r\n",
            JOIN_ATTEMPT_TIMEOUT_MS / 1000UL);

    uint32_t started_at = millis();
    uint32_t last_progress_log = started_at;

    while ((millis() - started_at) < JOIN_ATTEMPT_TIMEOUT_MS) {
      if (api.lorawan.njs.get()) {
        Serial.println("[JOIN] Join complete: network session is active");
        g_join_announced = true;
        return true;
      }

      if ((millis() - last_progress_log) >= 10000UL) {
        Serial.println("[JOIN] Waiting for accept...");
        last_progress_log = millis();
      }
      delay(JOIN_POLL_INTERVAL_MS);
    }

    if (api.lorawan.njs.get()) {
      Serial.println("[JOIN] Join complete after final state check");
      g_join_announced = true;
      return true;
    }

    if (g_join_event_seen) {
      Serial.printf("[JOIN] Attempt %u ended without session, callback=%s\r\n",
                    attempt, loramac_status_name(g_last_join_status));
    } else {
      Serial.printf("[JOIN] Attempt %u timed out: join started but no accept was observed\r\n",
                    attempt);
    }

    if (attempt < JOIN_MAX_ATTEMPTS) {
      Serial.printf("[JOIN] Retrying in %lu seconds...\r\n",
                    JOIN_RETRY_DELAY_MS / 1000UL);
      delay(JOIN_RETRY_DELAY_MS);
    }
  }

  return false;
}

static bool ensure_joined() {
  if (api.lorawan.njs.get()) {
    if (!g_join_announced) {
      Serial.println("[JOIN] Network already joined");
      g_join_announced = true;
    }
    return true;
  }

  if (run_join_attempts()) {
    return true;
  }

  Serial.println("[JOIN] FAILED: either credentials/config are wrong, device state was invalid, or RF/network accept is missing");
  return false;
}

static bool wait_for_send_result(uint32_t timeout_ms) {
  uint32_t started_at = millis();
  while (!g_send_event_seen && (millis() - started_at) < timeout_ms) {
    delay(50);
  }

  if (!g_send_event_seen) {
    Serial.println("[SEND] Timeout while waiting for send callback");
    return false;
  }

  if (g_last_send_status == RAK_LORAMAC_STATUS_OK) {
    Serial.println("[SEND] Uplink finished successfully");
    return true;
  }

  Serial.printf("[SEND] Uplink failed with status %s\r\n",
                loramac_status_name(g_last_send_status));
  return false;
}

static bool send_measurement_payload() {
  if (!g_bme_ready) {
    Serial.println("[SENS] BME280 is not ready, skipping payload send");
    return false;
  }

  digitalWrite(PWR_ENABLE_PIN, HIGH);
  delay(100);

  float temp_c = bme280_read_temperature();
  float humidity_pct = bme280_read_humidity();
  float o2_raw = read_o2_percent_raw();
  float vbat = read_battery_voltage();
  uint8_t bat_pct = battery_percent(vbat);

  PayloadV1 payload = make_payload(o2_raw, temp_c, humidity_pct, bat_pct);
  log_payload_preview(payload, o2_raw, temp_c, humidity_pct, vbat);

  g_send_event_seen = false;
  g_last_send_status = INT32_MIN;

  Serial.printf("[SEND] Requesting uplink on FPort %u (%s)\r\n",
                LORA_FPORT, LORA_CONFIRMED_UPLINK ? "confirmed" : "unconfirmed");

  if (!api.lorawan.send(sizeof(payload),
                        (uint8_t *)&payload,
                        LORA_FPORT,
                        LORA_CONFIRMED_UPLINK,
                        LORA_SEND_RETRY)) {
    Serial.println("[SEND] Start failed: stack did not accept the uplink request");
    return false;
  }

  return wait_for_send_result(15000UL);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200, RAK_AT_MODE);
  delay(1200);

  Serial.println();
  Serial.println("========================================");
  Serial.println("=== RAK3172 RUI O2 + BME280 SENSOR   ===");
  Serial.println("=== VS Code / Join-first build       ===");
  Serial.println("========================================");

  log_rui_versions();

  pinMode(PWR_ENABLE_PIN, OUTPUT);
  digitalWrite(PWR_ENABLE_PIN, HIGH);
  Serial.println("[PWR] Sensor rail enabled on PA8");

  Serial1.end();
  pinMode(PB6, INPUT);
  pinMode(PB7, INPUT);
  Serial.println("[UART1] Serial1 disabled to free PB6/PB7 for I2C");

  analogReadResolution(10);
  pinMode(BAT_ADC_PIN, INPUT);
  pinMode(O2_ADC_PIN, INPUT);
  Serial.println("[ADC] Battery on PB3, O2 on PB2");

  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  Wire.begin();
  Serial.println("[I2C] Wire ready on PB7/PB6");

  Serial.println("[BME] Initializing...");
  g_bme_ready = bme280_init();
  if (g_bme_ready) {
    float t = bme280_read_temperature();
    float h = bme280_read_humidity();
    Serial.printf("[BME] OK, test reading %.2f C / %.2f %%RH\r\n", t, h);
  } else {
    Serial.println("[BME] FAILED - join diagnostics can still run");
  }

  float vbat = read_battery_voltage();
  float o2 = read_o2_percent_raw();
  Serial.println("[SENS] Boot sanity check:");
  Serial.printf("  Battery: %.3f V (%u%%)\r\n", vbat, battery_percent(vbat));
  Serial.printf("  O2 raw:  %.2f %%\r\n", o2);

  if (!configure_lorawan()) {
    Serial.println("[INIT] LoRaWAN configuration failed");
    Serial.println("========================================");
    return;
  }

  Serial.printf("[MODE] Join diagnostics only: %s\r\n", JOIN_DIAGNOSTICS_ONLY ? "YES" : "NO");
  Serial.printf("[MODE] Low power sleep:       %s\r\n", ENABLE_LOW_POWER_SLEEP ? "YES" : "NO");
  Serial.println("========================================");
}

// ================= LOOP =================
void loop() {
  static uint32_t last_send_at = 0;

  if (!ensure_joined()) {
    delay(JOIN_RETRY_DELAY_MS);
    return;
  }

  if (JOIN_DIAGNOSTICS_ONLY) {
    Serial.println("[JOIN] Diagnostics mode active, no sensor payload will be sent");
    delay(5000);
    return;
  }

  uint32_t now = millis();
  if (last_send_at == 0 || (now - last_send_at) >= SEND_INTERVAL_MS) {
    last_send_at = now;
    send_measurement_payload();
    Serial.println("----------------------------------------");
  }

  if (ENABLE_LOW_POWER_SLEEP) {
    api.system.sleep.all(1000);
  } else {
    delay(1000);
  }
}
