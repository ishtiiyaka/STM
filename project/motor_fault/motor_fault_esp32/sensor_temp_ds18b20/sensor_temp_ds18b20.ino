/*
 ============================================================
  DS18B20 Temperature Sensor — ESP32 Standalone Sketch
  UET Lahore — Special Topics in Mechatronics, Spring 2026
 ============================================================
  Hardware:
    DS18B20  → GPIO2  (4.7 kΩ pullup to 3.3 V)

  Serial output (115200 baud) — CSV one line per sample:
    timestamp_ms, temp_C
 ============================================================
*/

#include <OneWire.h>
#include <DallasTemperature.h>

// ─── Pin Definition ───────────────────────────────────────
#define DS18B20_PIN   2     // GPIO2 — data line (4.7 kΩ to 3.3 V)

// ─── Sensor Objects ───────────────────────────────────────
OneWire           oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

// ─── State ────────────────────────────────────────────────
float         temperature    = 25.0f;
unsigned long lastTempRequest = 0;
// DS18B20 at 9-bit resolution needs ~94 ms to convert.
// We poll every 100 ms so we never read stale data.
#define TEMP_INTERVAL_MS  100

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ── DS18B20 init ──────────────────────────────────────
  tempSensor.begin();

  // 9-bit resolution  →  0.5 °C steps, ~94 ms conversion time
  // (use 12-bit for 0.0625 °C steps but you must wait ~750 ms)
  tempSensor.setResolution(9);

  // Non-blocking: requestTemperatures() returns immediately;
  // we read the result after the conversion window has elapsed.
  tempSensor.setWaitForConversion(false);

  // Kick off the very first conversion
  tempSensor.requestTemperatures();
  lastTempRequest = millis();

  // ── CSV Header ────────────────────────────────────────
  Serial.println(F("timestamp_ms,temp_C"));
}

void loop() {
  unsigned long now = millis();

  // Read result once conversion window has elapsed, then request next
  if (now - lastTempRequest >= TEMP_INTERVAL_MS) {
    float t = tempSensor.getTempCByIndex(0);

    // Sanity-check: reject disconnected / out-of-range readings
    if (t != DEVICE_DISCONNECTED_C && t > -40.0f && t < 150.0f) {
      temperature = t;
    }

    // Print CSV line
    Serial.print(now);
    Serial.print(',');
    Serial.println(temperature, 2);   // 2 decimal places

    // Request next conversion
    tempSensor.requestTemperatures();
    lastTempRequest = now;
  }
}
