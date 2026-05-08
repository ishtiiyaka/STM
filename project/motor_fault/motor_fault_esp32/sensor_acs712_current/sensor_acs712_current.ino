/*
 ============================================================
  ACS712 Current Sensor (×1) — ESP32 Standalone Sketch
  UET Lahore — Special Topics in Mechatronics, Spring 2026
 ============================================================
  Hardware:
    ACS712-20A  → GPIO34 (Motor 1)  — ADC1_CH6, input-only
    Sensor VCC → 3.3 V
    ADC attenuation → 11 dB (0 – 3.3 V full range)

  Sensitivity:
    ACS712-20A  → 100 mV/A  (change to 0.185 for 5A, 0.066 for 30A)

  Serial output (115200 baud) — CSV one line per sample:
    timestamp_us, curr1_A
 ============================================================
*/

#include <Arduino.h>

// ─── Pin Definitions ──────────────────────────────────────
#define ACS712_M1_PIN   34      // Motor 1 current (ADC1_CH6)

// ─── ACS712 Calibration ───────────────────────────────────
// ACS712-20A: 100 mV/A  |  ACS712-5A: 185 mV/A  |  ACS712-30A: 66 mV/A
#define ACS712_SENS_VA  0.100f   // V per Ampere

// ─── ADC Config ───────────────────────────────────────────
#define ADC_MAX         4095.0f  // 12-bit
#define VCC_V           3.3f     // ADC reference with 11 dB attenuation

// ─── Sampling Config ──────────────────────────────────────
// 1000 µs → 1 kHz.  Single-sample per interval (no averaging = raw data).
#define SAMPLE_INTERVAL_US  1000UL   // 1 ms → 1000 Hz

// ─── Calibration Config ───────────────────────────────────
#define CAL_SAMPLES          1000    // 1000 × 1 ms = 1 s window
// FIXED: Increased from 0.015f to 0.050f to prevent infinite loop from ESP32 ADC noise
#define CAL_NOISE_THRESH_V   0.050f  

// ─── Zero-current offset (filled during calibration) ──────
float offset_m1_V = VCC_V / 2.0f;   // default midpoint; overwritten by calibration

unsigned long lastSample = 0;

// ─── Helpers ──────────────────────────────────────────────
inline float adcToVoltage(int raw) {
  return (raw / ADC_MAX) * VCC_V;
}

inline float voltsToCurrent(float v, float offset_V) {
  return (v - offset_V) / ACS712_SENS_VA;
}

// ─────────────────────────────────────────────────────────
void calibrateACS712() {
  bool calOK = false;

  while (!calOK) {
    Serial.println(F("# ACS712 calibration — motor OFF, no load for 1 second..."));

    // Accumulators
    double sum1 = 0.0;
    float  samples1[CAL_SAMPLES];

    for (int i = 0; i < CAL_SAMPLES; i++) {
      float v1 = adcToVoltage(analogRead(ACS712_M1_PIN));
      samples1[i] = v1;
      sum1 += v1;
      delayMicroseconds(1000);   // 1 ms → 1 kHz sample rate during calibration
    }

    float mean1 = (float)(sum1 / CAL_SAMPLES);

    // Compute stddev to detect motor activity during calibration window
    float var1 = 0.0f;
    for (int i = 0; i < CAL_SAMPLES; i++) {
      float d = samples1[i] - mean1;
      var1 += d * d;
    }
    float stddev1 = sqrtf(var1 / CAL_SAMPLES);

    if (stddev1 > CAL_NOISE_THRESH_V) {
      Serial.print(F("# Calibration REJECTED — M1 noise stddev="));
      Serial.print(stddev1 * 1000.0f, 1);
      Serial.println(F(" mV (motor may be running or ADC is noisy). Retrying..."));
      delay(500);
      continue;
    }

    offset_m1_V = mean1;
    calOK = true;
  }

  Serial.print(F("# Zero offset M1 = "));
  Serial.print(offset_m1_V * 1000.0f, 1);
  Serial.println(F(" mV"));

  Serial.print(F("# Zero offset M1 = "));
  Serial.print((offset_m1_V - VCC_V / 2.0f) / ACS712_SENS_VA, 4);
  Serial.println(F(" A  (residual from ideal mid)"));

  Serial.println(F("# Calibration OK — starting data stream."));
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);

  analogReadResolution(12);            // 12-bit ADC
  
  // FIXED: Core 3.x compatibility check for analog attenuation
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    analogSetPinAttenuation(ACS712_M1_PIN, ADC_11db);
  #else
    analogSetAttenuation(ADC_11db);      // 0–3.3 V full scale
  #endif

  Serial.println(F("\n# ACS712 Current Sensor (×1) — ESP32"));

  delay(200);   // let ADC settle after attenuation change
  calibrateACS712();

  // CSV header
  Serial.println(F("timestamp_us,curr1_A"));

  lastSample = micros();
}

// ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();

  if (now - lastSample >= SAMPLE_INTERVAL_US) {
    lastSample = now;

    // Single-sample read — no averaging = raw signal
    float v1 = adcToVoltage(analogRead(ACS712_M1_PIN));
    float c1 = voltsToCurrent(v1, offset_m1_V);

    Serial.print(now);
    Serial.print(',');
    Serial.println(c1, 4);
  }
}