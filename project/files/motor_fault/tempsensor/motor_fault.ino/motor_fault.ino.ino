#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

void setup(void) {
  // Start serial port at the baud rate specified in your guide
  Serial.begin(115200);
  Serial.println("DS18B20 Temperature Sensor Test");

  // Start up the library
  sensors.begin();
  
  // Set to 9-bit resolution as recommended in your troubleshooting section
  // This speeds up the conversion time, which is helpful for high-frequency data logging
  sensors.setResolution(9); 
}

void loop(void) { 
  // Send the command to get temperatures
  sensors.requestTemperatures(); 
  
  // Fetch the temperature in Celsius for the first sensor on the wire
  float tempC = sensors.getTempCByIndex(0);
  
  // Check if reading was successful (returns -127 if disconnected/missing resistor)
  if(tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data. Check wiring and 4.7k resistor!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" C");
  }
  
  // Wait a bit before the next reading
  delay(500); 
}