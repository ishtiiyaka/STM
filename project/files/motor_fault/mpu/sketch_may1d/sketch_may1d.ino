#include <Wire.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin();
  
  // Wake up the MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero to wake up the sensor
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 initialized. Reading data...");
}

void loop() {
  // Tell the sensor we want to read data starting from the Accelerometer X register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  // Request 14 bytes total (Accel X, Y, Z, Temp, Gyro X, Y, Z)
  // We will just read Accel and Gyro for this test
  Wire.requestFrom(MPU_ADDR, 14, true);  
  
  AcX = Wire.read()<<8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY = Wire.read()<<8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read()<<8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  // Skip the temperature registers (2 bytes) since we have the DS18B20 for that
  Wire.read(); 
  Wire.read();
  
  GyX = Wire.read()<<8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read()<<8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Print the raw Accelerometer values
  // Print the data in a format the Serial Plotter loves
  Serial.print("Accel_X:"); 
  Serial.print(AcX);
  Serial.print(","); // Comma separates the lines
  
  Serial.print("Accel_Y:"); 
  Serial.print(AcY);
  Serial.print(",");
  
  Serial.print("Accel_Z:"); 
  Serial.println(AcZ); // println adds the hidden 'enter' key at the end

  delay(5); // Faster delay (50ms) makes for a much smoother, cooler-looking graph!
}