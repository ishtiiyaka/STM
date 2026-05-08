/*
 ============================================================
  Motor Fault Detection System  —  ESP32 Port
  UET Lahore — Special Topics in Mechatronics, Spring 2026
 ============================================================
  Hardware (ESP32):
    MPU6050        → I2C  (SDA=GPIO21, SCL=GPIO22) — raw Wire.h, no library
    DS18B20        → GPIO2   (4.7 kΩ pullup to 3.3 V)
    ACS712 Motor1  → GPIO34  (ADC1_CH6 — input-only, healthy reference)
    ACS712 Motor2  → GPIO35  (ADC1_CH7 — input-only, test/fault motor)
    Potentiometer  → GPIO32  (ADC1_CH4 — 3-position: selects sample rate)

  ESP32 ADC notes:
    • 12-bit resolution  → 0–4095 counts
    • Reference voltage  → 3.3 V  (NOT 5 V)
    • ADC2 pins conflict with Wi-Fi; always use ADC1 pins (GPIO32–39)
    • Built-in attenuation set to 11 dB for full 0–3.3 V range

  Serial output (115200 baud) — CSV one line per sample:
    timestamp_ms, curr1_A, curr2_A, temp_C,
    ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, sampleRate_Hz

  NOTE: Change ACS712_SENS to match your module variant:
    5A  module → 0.185
    20A module → 0.100
    30A module → 0.066
 ============================================================
*/

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pin Definitions (ESP32) ──────────────────────────────
#define DS18B20_PIN      2      // DS18B20 data pin (GPIO2)
#define ACS712_M1_PIN   34      // Healthy motor current (ADC1_CH6)
#define ACS712_M2_PIN   35      // Test motor current   (ADC1_CH7)
#define POT_PIN         32      // Potentiometer        (ADC1_CH4)

// ─── I2C Pins (ESP32 default) ─────────────────────────────
#define I2C_SDA         21
#define I2C_SCL         22

// ─── ACS712 Calibration ───────────────────────────────────
// ESP32 logic is 3.3 V — measure actual rail with a multimeter
#define VCC_MV          3300.0f   // millivolts (adjust if needed)
#define ACS712_SENS     0.100f    // V/A — see note above
// Offset voltage at 0 A = VCC/2.  Measured at startup.
float acs712_offset_m1 = 1.65f;
float acs712_offset_m2 = 1.65f;

// ─── ADC Resolution (ESP32 = 12-bit) ─────────────────────
#define ADC_MAX         4095.0f

// ─── MPU6050 ──────────────────────────────────────────────
#define MPU_ADDR       0x68
// Accel range ±8g   → 4096 LSB/g
// Gyro  range ±500° → 65.5  LSB/°/s
#define ACCEL_SCALE   4096.0f
#define GYRO_SCALE    65.5f

int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

// ─── DS18B20 ──────────────────────────────────────────────
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
float temperature   = 25.0f;
unsigned long lastTempRequest = 0;

// ─── Timing ───────────────────────────────────────────────
unsigned long lastSample     = 0;
int           sampleInterval_ms = 10;   // default 100 Hz

// ─────────────────────────────────────────────────────────
void setupMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  // Accel range ±8g  (register 0x1C, bits 4:3 = 10)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission(true);

  // Gyro range ±500°/s  (register 0x1B, bits 4:3 = 01)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission(true);

  // DLPF bandwidth 44 Hz (register 0x1A, DLPF_CFG = 3)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x03);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

  ax_r = (Wire.read() << 8) | Wire.read();
  ay_r = (Wire.read() << 8) | Wire.read();
  az_r = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // skip MPU internal temp registers
  gx_r = (Wire.read() << 8) | Wire.read();
  gy_r = (Wire.read() << 8) | Wire.read();
  gz_r = (Wire.read() << 8) | Wire.read();
}

// Oversample analog read (average 20 readings) for noise reduction
// ESP32 ADC: 12-bit (0–4095), Vref = 3.3 V
float readCurrentA(int pin, float offset_V) {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  float avgADC  = sum / 20.0f;
  float voltage = (avgADC / ADC_MAX) * (VCC_MV / 1000.0f);
  float current = (voltage - offset_V) / ACS712_SENS;
  // Absolute value — negative only means reversed current direction
  return fabsf(current);
}

// Calibrate ACS712 zero offset (call with motors OFF)
void calibrateACS712() {
  long sum1 = 0, sum2 = 0;
  for (int i = 0; i < 200; i++) {
    sum1 += analogRead(ACS712_M1_PIN);
    sum2 += analogRead(ACS712_M2_PIN);
    delay(1);
  }
  acs712_offset_m1 = (sum1 / 200.0f / ADC_MAX) * (VCC_MV / 1000.0f);
  acs712_offset_m2 = (sum2 / 200.0f / ADC_MAX) * (VCC_MV / 1000.0f);
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── ESP32 ADC config ──────────────────────────────────
  // 12-bit resolution (default on ESP32, explicit for clarity)
  analogReadResolution(12);
  // 11 dB attenuation → full 0–3.3 V input range
  analogSetAttenuation(ADC_11db);

  // ── I2C (custom pins) ─────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);   // 400 kHz fast mode

  // ── MPU6050 ───────────────────────────────────────────
  setupMPU6050();

  // ── DS18B20 ───────────────────────────────────────────
  tempSensor.begin();
  tempSensor.setResolution(9);             // 9-bit ≈ 94 ms conversion
  tempSensor.setWaitForConversion(false);  // non-blocking / async
  tempSensor.requestTemperatures();        // kick off first conversion
  lastTempRequest = millis();

  // ── ACS712 zero calibration ───────────────────────────
  // Motors must be OFF before power-on / upload!
  Serial.println(F("# Calibrating ACS712 offsets (motors must be OFF)..."));
  delay(500);
  calibrateACS712();
  Serial.print(F("# ACS712 offsets: M1="));
  Serial.print(acs712_offset_m1, 4);
  Serial.print(F(" V   M2="));
  Serial.println(acs712_offset_m2, 4);

  // ── CSV Header ────────────────────────────────────────
  Serial.println(F("timestamp_ms,curr1_A,curr2_A,temp_C,"
                   "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,sampleRate_Hz"));
}

void loop() {
  // ── Select sample rate via potentiometer ──────────────
  // Pot position   0–1364  →  100  Hz (10 ms)
  //              1365–2729  →  500  Hz ( 2 ms)
  //              2730–4095  → 1000  Hz ( 1 ms)
  // (12-bit range 0–4095 split into three equal thirds)
  int potVal = analogRead(POT_PIN);
  if      (potVal < 1365) sampleInterval_ms = 10;
  else if (potVal < 2730) sampleInterval_ms =  2;
  else                    sampleInterval_ms =  1;

  unsigned long now = millis();

  // ── Async DS18B20 read (every 100 ms) ─────────────────
  if (now - lastTempRequest >= 100) {
    float t = tempSensor.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C && t > -40.0f && t < 150.0f) {
      temperature = t;
    }
    tempSensor.requestTemperatures();
    lastTempRequest = now;
  }

  // ── Sample at selected rate ───────────────────────────
  if (now - lastSample >= (unsigned long)sampleInterval_ms) {
    lastSample = now;

    readMPU6050();
    float c1 = readCurrentA(ACS712_M1_PIN, acs712_offset_m1);
    float c2 = readCurrentA(ACS712_M2_PIN, acs712_offset_m2);

    float ax = ax_r / ACCEL_SCALE;
    float ay = ay_r / ACCEL_SCALE;
    float az = az_r / ACCEL_SCALE;
    float gx = gx_r / GYRO_SCALE;
    float gy = gy_r / GYRO_SCALE;
    float gz = gz_r / GYRO_SCALE;
    int   fs = 1000 / sampleInterval_ms;

    // ── Print CSV line ──
    Serial.print(now);              Serial.print(',');
    Serial.print(c1,   4);          Serial.print(',');
    Serial.print(c2,   4);          Serial.print(',');
    Serial.print(temperature, 2);   Serial.print(',');
    Serial.print(ax,   5);          Serial.print(',');
    Serial.print(ay,   5);          Serial.print(',');
    Serial.print(az,   5);          Serial.print(',');
    Serial.print(gx,   4);          Serial.print(',');
    Serial.print(gy,   4);          Serial.print(',');
    Serial.print(gz,   4);          Serial.print(',');
    Serial.println(fs);
  }
}
