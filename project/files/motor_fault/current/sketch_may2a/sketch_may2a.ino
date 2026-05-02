/*
 ============================================================
  Dual ACS712 20A Plotter Test
 ============================================================
  Hardware Wiring:
    Both ACS712 VCC -> Arduino 5V
    Both ACS712 GND -> Arduino GND
    ACS712 Motor 1 OUT -> Arduino A0
    ACS712 Motor 2 OUT -> Arduino A1
 ============================================================
*/

// ─── Pin & Calibration Definitions ────────────────────────
#define ACS712_M1_PIN   A0        // Sensor 1 input
#define ACS712_M2_PIN   A1        // Sensor 2 input
#define VCC_MV          4980.0f   // Board voltage in millivolts
#define ACS712_SENS     0.100f    // Sensitivity for 20A module (100mV/A)

// Individual offsets for each sensor
float offset_m1 = 2.5f; 
float offset_m2 = 2.5f;

// ─── Read and Calculate Current ───────────────────────────
float readCurrentA(int pin, float zero_offset) {
  long sum = 0;
  int samples = 20; 
  
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  
  float avgADC = sum / (float)samples;
  float current_voltage = (avgADC / 1023.0f) * (VCC_MV / 1000.0f);
  float current = (current_voltage - zero_offset) / ACS712_SENS;
  
  return abs(current);
}

// ─── Auto-Calibrate Zero Point ────────────────────────────
void calibrateBothSensors() {
  long sum1 = 0;
  long sum2 = 0;
  int samples = 200;
  
  for (int i = 0; i < samples; i++) {
    sum1 += analogRead(ACS712_M1_PIN);
    sum2 += analogRead(ACS712_M2_PIN);
    delay(2);
  }
  
  offset_m1 = (sum1 / (float)samples / 1023.0f) * (VCC_MV / 1000.0f);
  offset_m2 = (sum2 / (float)samples / 1023.0f) * (VCC_MV / 1000.0f);
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  
  // Give a moment to stabilize
  delay(1000);
  
  // Calibrate while motors/loads are OFF
  calibrateBothSensors();
  
  // Print the legend for the Serial Plotter exactly ONCE
  // (Do not add spaces after the comma)
  Serial.println("Motor_1_Amps,Motor_2_Amps");
}

void loop() {
  // Read both sensors using their specific offsets
  float amps1 = readCurrentA(ACS712_M1_PIN, offset_m1);
  float amps2 = readCurrentA(ACS712_M2_PIN, offset_m2);
  
  // Print in the exact format the Plotter expects: "Value1,Value2\n"
  Serial.print(amps1, 3);
  Serial.print(",");
  Serial.println(amps2, 3);
  
  // 20ms delay gives a smooth 50Hz update rate for the graph
  delay(2000); 
}