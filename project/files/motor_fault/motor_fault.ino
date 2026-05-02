/*
 ============================================================
  Motor Fault Detection System
  UET Lahore — Special Topics in Mechatronics, Spring 2026
 ============================================================
  Hardware:
    MPU6050        → I2C  (SDA=A4, SCL=A5)  — raw Wire.h, no library
    DS18B20        → D2   (4.7kΩ pullup to 5V)
    ACS712 Motor1  → A0   (healthy reference motor)
    ACS712 Motor2  → A1   (test/fault motor)
    Potentiometer  → A2   (3-position: selects sample rate)

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

// ─── Pin Definitions ──────────────────────────────────────
#define DS18B20_PIN      2     // DS18B20 data pin
#define ACS712_M1_PIN   A0     // Healthy motor current
#define ACS712_M2_PIN   A1     // Test motor current
#define POT_PIN         A2     // Potentiometer for sample rate

// ─── ACS712 Calibration ───────────────────────────────────
// Measure actual Vcc with a multimeter and enter below
#define VCC_MV          4980.0f   // millivolts (adjust if needed)
#define ACS712_SENS     0.100f    // V/A — see note above
// Offset voltage at 0A = VCC/2. We measure it at startup.
float acs712_offset_m1 = 2.5f;
float acs712_offset_m2 = 2.5f;

// ─── MPU6050 ──────────────────────────────────────────────
#define MPU_ADDR       0x68
// Accel range ±8g  → 4096 LSB/g
// Gyro  range ±500° → 65.5 LSB/°/s
#define ACCEL_SCALE   4096.0f
#define GYRO_SCALE    65.5f

int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

// ─── DS18B20 ──────────────────────────────────────────────
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
float temperature = 25.0f;
unsigned long lastTempRequest = 0;

// ─── Timing ───────────────────────────────────────────────
unsigned long lastSample = 0;
int sampleInterval_ms = 10;  // default 100 Hz

// ─────────────────────────────────────────────────────────
void setupMPU6050() {
  // Wake up
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
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax_r = (Wire.read() << 8) | Wire.read();
  ay_r = (Wire.read() << 8) | Wire.read();
  az_r = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip MPU internal temp
  gx_r = (Wire.read() << 8) | Wire.read();
  gy_r = (Wire.read() << 8) | Wire.read();
  gz_r = (Wire.read() << 8) | Wire.read();
}

// Oversample analog read (average 20 readings) for noise reduction
float readCurrentA(int pin, float offset_V) {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  float avgADC  = sum / 20.0f;
  float voltage = (avgADC / 1023.0f) * (VCC_MV / 1000.0f);
  float current = (voltage - offset_V) / ACS712_SENS;
  // Take absolute value; negative only means reversed current direction
  return abs(current);
}

// Calibrate ACS712 zero offset (call with motors OFF)
void calibrateACS712() {
  long sum1 = 0, sum2 = 0;
  for (int i = 0; i < 200; i++) {
    sum1 += analogRead(ACS712_M1_PIN);
    sum2 += analogRead(ACS712_M2_PIN);
    delay(1);
  }
  acs712_offset_m1 = (sum1 / 200.0f / 1023.0f) * (VCC_MV / 1000.0f);
  acs712_offset_m2 = (sum2 / 200.0f / 1023.0f) * (VCC_MV / 1000.0f);
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // I2C fast mode for MPU6050

  // --- MPU6050 ---
  setupMPU6050();

  // --- DS18B20 ---
  tempSensor.begin();
  tempSensor.setResolution(9);            // 9-bit = ~94ms conversion time
  tempSensor.setWaitForConversion(false); // async mode
  tempSensor.requestTemperatures();       // kick off first conversion
  lastTempRequest = millis();

  // --- ACS712 zero calibration ---
  // Make sure motors are OFF before uploading!
  Serial.println(F("# Calibrating ACS712 offsets (motors must be OFF)..."));
  delay(500);
  calibrateACS712();
  Serial.print(F("# ACS712 offsets: M1="));
  Serial.print(acs712_offset_m1, 4);
  Serial.print(F("V  M2="));
  Serial.println(acs712_offset_m2, 4);

  // --- CSV Header ---
  Serial.println(F("timestamp_ms,curr1_A,curr2_A,temp_C,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,sampleRate_Hz"));
}

void loop() {
  // ── Select sample rate via potentiometer ──
  // Pot position 0–341 → 100 Hz (10 ms)
  //              342–682 → 500 Hz (2 ms)
  //              683–1023 → 1000 Hz (1 ms)
  int potVal = analogRead(POT_PIN);
  if      (potVal < 341) sampleInterval_ms = 10;
  else if (potVal < 682) sampleInterval_ms = 2;
  else                   sampleInterval_ms = 1;

  unsigned long now = millis();

  // ── Async DS18B20 read (every 100 ms) ──
  if (now - lastTempRequest >= 100) {
    float t = tempSensor.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C && t > -40.0f && t < 150.0f) {
      temperature = t;
    }
    tempSensor.requestTemperatures();
    lastTempRequest = now;
  }

  // ── Sample at selected rate ──
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

    // Print CSV line
    Serial.print(now);         Serial.print(',');
    Serial.print(c1, 4);       Serial.print(',');
    Serial.print(c2, 4);       Serial.print(',');
    Serial.print(temperature, 2); Serial.print(',');
    Serial.print(ax, 5);       Serial.print(',');
    Serial.print(ay, 5);       Serial.print(',');
    Serial.print(az, 5);       Serial.print(',');
    Serial.print(gx, 4);       Serial.print(',');
    Serial.print(gy, 4);       Serial.print(',');
    Serial.print(gz, 4);       Serial.print(',');
    Serial.println(fs);
  }
}
