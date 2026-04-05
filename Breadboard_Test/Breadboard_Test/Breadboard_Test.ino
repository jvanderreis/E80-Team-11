#include <math.h>

// --- E80 PROTOBOARD PIN DEFINITIONS ---
const int MUX_S0 = 33;
const int MUX_S1 = 32;
const int MUX_S2 = 31;
const int MUX_S3 = 28;
const int MUX_SIG = A3; 

const int ANEMOMETER_PIN = A0; 
const int THERM_WATER = A1;
const int THERM_AIR = A2;


// --- THERMISTOR CIRCUIT CONSTANTS ---
// Values based on the 5V, 100k thermistor, and op-amp mapping
const float BETA = 3950.0;
const float R_NOMINAL = 100000.0; // 100k ohms at 25C
const float T_NOMINAL = 298.15;   // 25C in Kelvin
const float V_SOURCE = 5.0;       // Voltage divider source
const float R_FIXED = 100000.0;   // Voltage divider bottom resistor
const float GAIN = 2.7;           // Op-amp gain (Rf/Rin)
const float OFFSET_CONST = 7.63125; // Vref * (1 + Rf/Rin) -> 2.0625V * 3.7

volatile unsigned long windPulseCount = 0;
unsigned long previousMillis = 0;
unsigned long previousPulseCount = 0;

void countWindPulse() {
  windPulseCount++;
}

// Helper function to reverse-calculate temp from the op-amp output
float calculateTemperature(float vOut) {
  // 1. Reverse the Inverting Amplifier to find Vin
  // Vout = Offset - (Vin * Gain)  -->  Vin = (Offset - Vout) / Gain
  float vIn = (OFFSET_CONST - vOut) / GAIN;

  // Protect against division by zero or negative log errors if a wire pulls out
  if (vIn <= 0.01 || vIn >= V_SOURCE) return -999.0; 

  // 2. Reverse the Voltage Divider to find Thermistor Resistance (Rt)
  // Vin = V_SOURCE * (R_FIXED / (Rt + R_FIXED))
  float rTherm = R_FIXED * ((V_SOURCE / vIn) - 1.0);

  // 3. Steinhart-Hart (Beta) Equation for Temperature
  float steinhart = rTherm / R_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);               // ln(R/Ro)
  steinhart /= BETA;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / T_NOMINAL;             // + (1/To)
  steinhart = 1.0 / steinhart;              // Invert
  
  float tCelsius = steinhart - 273.15;      // Convert Kelvin to Celsius
  return tCelsius;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Mux Control Pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countWindPulse, FALLING);

  // Optional: Set ADC resolution to 10 or 12 bits depending on your needs. 
  // Teensy 4.0 defaults to 10-bit (1024), but can do 12-bit (4096) for smoother temp curves.
  analogReadResolution(10); 

  Serial.println("E80 Breadboard Checkoff: Sensor Package Online");
}

void loop() {
  Serial.println("\n--- Sun Compass (Phototransistors) ---");
  for (int i = 0; i < 16; i++) {
    
    // Explicit bitwise logic ensuring strict HIGH/LOW states
    digitalWrite(MUX_S0, (i & 0x01) ? HIGH : LOW);
    digitalWrite(MUX_S1, (i & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_S2, (i & 0x04) ? HIGH : LOW);
    digitalWrite(MUX_S3, (i & 0x08) ? HIGH : LOW);

    // Give the MUX internal switches time to physically latch
    delayMicroseconds(500);

    int lightVal = analogRead(MUX_SIG);
    float voltage = lightVal * (3.3 / 1023.0); // Adjust to 4095.0 if using 12-bit resolution
    
    Serial.print("Ch "); Serial.print(i); 
    Serial.print(": "); Serial.print(voltage, 2); Serial.print("V\t");
    if ((i + 1) % 4 == 0) Serial.println(); 
  }

Serial.println("--- Anemometer ---");
  
  // Safely grab the volatile count by disabling interrupts briefly
  noInterrupts();
  unsigned long currentPulseCount = windPulseCount;
  interrupts();

  // --- RESTORED: Print the total magnet passes ---
  Serial.print("Total Magnet Passes: "); 
  Serial.println(currentPulseCount);

  unsigned long currentMillis = millis();
  unsigned long timeDelta = currentMillis - previousMillis;
  
  // Calculate pulses per second (Hz)
  float frequency = 0.0;
  if (timeDelta > 0) {
    unsigned long pulseDelta = currentPulseCount - previousPulseCount;
    frequency = (pulseDelta * 1000.0) / timeDelta; 
  }

  Serial.print("Frequency (Hz): "); 
  Serial.println(frequency, 2);

  // Update tracking variables
  previousPulseCount = currentPulseCount;
  previousMillis = currentMillis;
  
  Serial.println("--- Thermistors ---");
  float airVoltage = analogRead(THERM_AIR) * (3.3 / 1023.0);
  float waterVoltage = analogRead(THERM_WATER) * (3.3 / 1023.0);
  
  float airTempC = calculateTemperature(airVoltage);
  float waterTempC = calculateTemperature(waterVoltage);
  
  Serial.print("Air Temp:   "); 
  Serial.print(airVoltage, 2); Serial.print("V  |  ");
  Serial.print(airTempC, 2); Serial.println(" °C");
  
  Serial.print("Water Temp: "); 
  Serial.print(waterVoltage, 2); Serial.print("V  |  ");
  Serial.print(waterTempC, 2); Serial.println(" °C");

  Serial.println("=====================================");
  delay(1000); 
}