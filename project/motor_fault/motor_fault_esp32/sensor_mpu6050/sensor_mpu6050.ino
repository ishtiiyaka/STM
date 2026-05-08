/*
 ============================================================
  MPU6050 IMU Sensor — ESP32 Standalone Sketch
  UET Lahore — Special Topics in Mechatronics, Spring 2026
 ============================================================
  Hardware:
    MPU6050  → I2C  (SDA = GPIO21, SCL = GPIO22)
               AD0 pin → GND  (I2C address = 0x68)
               VCC → 3.3 V

  Raw Wire.h only — no external MPU6050 library needed.

  Configured ranges:
    Accelerometer  →  ±8 g    (4096 LSB/g)
    Gyroscope      →  ±500°/s (65.5  LSB/°/s)
    DLPF           →  188 Hz  (register 0x1A, cfg = 1)
    SMPLRT_DIV     →  0       (register 0x19, full 1 kHz ODR)

  Serial output (921600 baud) — CSV one line per sample:
    timestamp_us, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps

  Gyro bias calibration runs automatically at startup:
    Keep sensor stationary until "Calibration OK" is printed.
 ============================================================
*/

#include <Wire.h>

// ─── I2C Pins (ESP32 default) ─────────────────────────────
#define I2C_SDA       21
#define I2C_SCL       22

// ─── MPU6050 I2C Address ──────────────────────────────────
// AD0 = GND → 0x68 | AD0 = 3.3V → 0x69
#define MPU_ADDR      0x68

// ─── Scaling Factors ──────────────────────────────────────
// Accel ±8g   → sensitivity = 4096 LSB/g
// Gyro  ±500° → sensitivity = 65.5  LSB/°/s
#define ACCEL_SCALE   4096.0f
#define GYRO_SCALE    65.5f

// ─── Raw register values ──────────────────────────────────
int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

// ─── Gyro bias offsets (filled during calibration) ────────
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

// ─── Sample rate ──────────────────────────────────────────
// Use micros() for 1 µs resolution at 1 kHz (1000 µs interval)
#define SAMPLE_INTERVAL_US  1000UL   // 1 ms → 1000 Hz

unsigned long lastSample = 0;

// ─────────────────────────────────────────────────────────
void setupMPU6050() {
  // ── Wake up (clear sleep bit in PWR_MGMT_1, reg 0x6B) ──
  // Use PLL with X-axis gyro as clock source (more stable than internal 8 MHz)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);   // CLKSEL = 1 → PLL with X gyro ref
  Wire.endTransmission(true);

  // ── Sample Rate Divider (reg 0x19) ──────────────────────
  // ODR = Gyro_Output_Rate / (1 + SMPLRT_DIV)
  // With DLPF_CFG >= 1, Gyro_Output_Rate = 1 kHz
  // SMPLRT_DIV = 0 → ODR = 1 kHz / 1 = 1000 Hz ✓
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(0x00);   // No division — full 1 kHz output
  Wire.endTransmission(true);

  // ── Digital Low-Pass Filter: 188 Hz BW (reg 0x1A, DLPF_CFG = 1) ──
  // CFG=1 → Accel BW=184 Hz, Gyro BW=188 Hz, ODR=1 kHz
  // Passes motor fundamentals (17–133 Hz) and first harmonics up to 266 Hz
  // Previous setting CFG=3 (44 Hz BW) was cutting off the motor signal entirely
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x01);   // DLPF_CFG = 1 → 188 Hz bandwidth
  Wire.endTransmission(true);

  // ── Accelerometer config: ±8g (reg 0x1C, AFS_SEL bits 4:3 = 10) ──
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  // ── Gyroscope config: ±500°/s (reg 0x1B, FS_SEL bits 4:3 = 01) ──
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);
}

// ─────────────────────────────────────────────────────────
void calibrateGyro() {
  // Collect GYRO_CAL_SAMPLES stationary readings and compute the mean bias.
  // The offsets are subtracted from every subsequent reading in readMPU6050().
  //
  // Stability gate: the standard deviation of gz must stay below
  // GYRO_CAL_NOISE_THRESH raw LSB; if the sensor moves during the window
  // the calibration restarts automatically.
  const int   GYRO_CAL_SAMPLES      = 1000;   // 1000 × 2 ms = 2 s window
  const float GYRO_CAL_NOISE_THRESH = 80.0f;  // ~1.2 °/s peak; rejects hand-shake

  bool calOK = false;

  while (!calOK) {
    Serial.println(F("# Gyro calibration — keep sensor STATIONARY for 2 seconds..."));

    long  sum_gx = 0, sum_gy = 0, sum_gz = 0;
    // Store gz samples on the heap to avoid blowing the stack
    int16_t* gz_buf = (int16_t*)malloc(GYRO_CAL_SAMPLES * sizeof(int16_t));

    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (bool)true);

      int16_t rx = (Wire.read() << 8) | Wire.read();
      int16_t ry = (Wire.read() << 8) | Wire.read();
      int16_t rz = (Wire.read() << 8) | Wire.read();

      sum_gx += rx;
      sum_gy += ry;
      sum_gz += rz;
      if (gz_buf) gz_buf[i] = rz;

      delay(2);  // 1000 samples × 2 ms = 2 s total
    }

    float mean_gz = sum_gz / (float)GYRO_CAL_SAMPLES;

    // Compute std-dev of gz to detect motion during calibration window
    float variance = 0.0f;
    if (gz_buf) {
      for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        float d = gz_buf[i] - mean_gz;
        variance += d * d;
      }
      free(gz_buf);
    }
    float stddev_gz = sqrt(variance / GYRO_CAL_SAMPLES);

    if (stddev_gz > GYRO_CAL_NOISE_THRESH) {
      Serial.print(F("# Calibration REJECTED — sensor moved (gz stddev="));
      Serial.print(stddev_gz, 1);
      Serial.println(F(" LSB). Retrying in 0.5 s..."));
      delay(500);
      continue;
    }

    gx_offset = sum_gx / (float)GYRO_CAL_SAMPLES;
    gy_offset = sum_gy / (float)GYRO_CAL_SAMPLES;
    gz_offset = mean_gz;
    calOK = true;
  }

  Serial.print(F("# Gyro bias (raw LSB): GX="));
  Serial.print(gx_offset, 1);
  Serial.print(F("  GY="));
  Serial.print(gy_offset, 1);
  Serial.print(F("  GZ="));
  Serial.println(gz_offset, 1);

  Serial.print(F("# Gyro bias (°/s):     GX="));
  Serial.print(gx_offset / GYRO_SCALE, 3);
  Serial.print(F("  GY="));
  Serial.print(gy_offset / GYRO_SCALE, 3);
  Serial.print(F("  GZ="));
  Serial.println(gz_offset / GYRO_SCALE, 3);

  Serial.println(F("# Calibration OK — starting data stream."));
}

// ─────────────────────────────────────────────────────────
void readMPU6050() {
  // Point to first data register (ACCEL_XOUT_H = 0x3B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Burst-read 14 bytes:
  // [AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L,
  //  TEMP_H, TEMP_L,
  //  GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L]
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true);

  ax_r = (Wire.read() << 8) | Wire.read();
  ay_r = (Wire.read() << 8) | Wire.read();
  az_r = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // discard MPU internal temperature

  // Read raw gyro and subtract calibrated bias offsets (in raw LSB units)
  gx_r = ((Wire.read() << 8) | Wire.read()) - (int16_t)gx_offset;
  gy_r = ((Wire.read() << 8) | Wire.read()) - (int16_t)gy_offset;
  gz_r = ((Wire.read() << 8) | Wire.read()) - (int16_t)gz_offset;
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);  // 921600 baud — required for 1 kHz × ~50 chars/line throughput

  // ── I2C bus ──────────────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);   // 400 kHz fast-mode

  // ── MPU6050 configuration ────────────────────────────
  setupMPU6050();

  // Brief sanity check — read WHO_AM_I register (0x75)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (bool)true);
  uint8_t whoAmI = Wire.read();
  // WHO_AM_I values:
  //   0x68 → genuine InvenSense MPU6050
  //   0x70 → common MPU6050 clone (GY-521 variant) — register-compatible
  bool mpuOK = (whoAmI == 0x68 || whoAmI == 0x70);
  Serial.print(F("# MPU6050 WHO_AM_I = 0x"));
  Serial.print(whoAmI, HEX);
  Serial.println(mpuOK ? F("  ✓ OK") : F("  ✗ NOT FOUND — check wiring"));

  if (!mpuOK) {
    Serial.println(F("# FATAL: MPU6050 not detected. Halting."));
    while (true) delay(1000);
  }

  // ── Gyro bias calibration ─────────────────────────────
  delay(100);   // let gyro settle after power-on
  calibrateGyro();

  // ── CSV Header ────────────────────────────────────────
  // timestamp_us: micros() timestamp for precise inter-sample timing
  Serial.println(F("timestamp_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps"));

  lastSample = micros();
}

void loop() {
  unsigned long now = micros();

  // Use micros() for 1 µs resolution timing at 1 kHz
  // millis() only has 1 ms resolution — at 1 kHz every sample would be
  // right on the edge of the timer tick, causing ±1 ms jitter (100% error).
  if (now - lastSample >= SAMPLE_INTERVAL_US) {
    lastSample = now;

    readMPU6050();

    // Convert raw counts to physical units
    float ax = ax_r / ACCEL_SCALE;
    float ay = ay_r / ACCEL_SCALE;
    float az = az_r / ACCEL_SCALE;
    float gx = gx_r / GYRO_SCALE;
    float gy = gy_r / GYRO_SCALE;
    float gz = gz_r / GYRO_SCALE;

    // Print CSV line
    Serial.print(now);       Serial.print(',');
    Serial.print(ax, 5);     Serial.print(',');
    Serial.print(ay, 5);     Serial.print(',');
    Serial.print(az, 5);     Serial.print(',');
    Serial.print(gx, 4);     Serial.print(',');
    Serial.print(gy, 4);     Serial.print(',');
    Serial.println(gz, 4);
  }
}
