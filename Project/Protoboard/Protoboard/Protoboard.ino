#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// --- E80 PINOUT & TIMING DEFINITIONS ---
#include <Pinouts.h> 
#include <TimingOffsets.h>

// E80 Framework Libraries
#include <DataSource.h> 
#include <Logger.h>
#include <Printer.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <Adafruit_GPS.h>

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
const float BETA = 3950.0;
const float R_NOMINAL = 100000.0; 
const float T_NOMINAL = 298.15;
const float V_SOURCE = 5.0;       
const float R_FIXED = 100000.0;
const float GAIN = 2.7;           
const float OFFSET_CONST = 7.63125;

// --- ANEMOMETER CALIBRATION CONSTANTS ---
const float ANEM_SLOPE = 2.1587;
const float ANEM_INTERCEPT = 1.2119;

// --- ANEMOMETER TRACKING (HIGH-RES LOGIC) ---
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulsePeriod = 0;

void countWindPulse() {
  unsigned long currentPulseTime = millis();
  if (currentPulseTime - lastPulseTime > 5) {
    pulsePeriod = currentPulseTime - lastPulseTime;
    lastPulseTime = currentPulseTime;
  }
}

float calculateTemperature(float vOut) {
  float vIn = (OFFSET_CONST - vOut) / GAIN;
  if (vIn <= 0.01 || vIn >= V_SOURCE) return -999.0;
  float rTherm = R_FIXED * ((V_SOURCE / vIn) - 1.0);
  float steinhart = rTherm / R_NOMINAL;     
  steinhart = log(steinhart);
  steinhart /= BETA;                        
  steinhart += 1.0 / T_NOMINAL;
  steinhart = 1.0 / steinhart;              
  return steinhart - 273.15; 
}

// ==============================================================================
// 1. CUSTOM E80 DATASOURCE CLASSES FOR BREADBOARD
// ==============================================================================

class SundialSamplerA : public DataSource {
  public:
    float ch[8];
    SundialSamplerA() : DataSource("ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7", "float,float,float,float,float,float,float,float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      for(int i = 0; i < 8; i++) data_ptr[i] = ch[i];
      return idx + 8 * sizeof(float);
    }
};

class SundialSamplerB : public DataSource {
  public:
    float ch[8];
    SundialSamplerB() : DataSource("ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15", "float,float,float,float,float,float,float,float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      for(int i = 0; i < 8; i++) data_ptr[i] = ch[i];
      return idx + 8 * sizeof(float);
    }
};

class WeatherSampler : public DataSource {
  public:
    float wind_vel; 
    float therm_air;
    float therm_water;
    WeatherSampler() : DataSource("wind_vel,therm_air,therm_water", "float,float,float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      data_ptr[0] = wind_vel;
      data_ptr[1] = therm_air;
      data_ptr[2] = therm_water;
      return idx + 3 * sizeof(float);
    }
};

// ==============================================================================
// 2. GLOBAL VARIABLES & TIMING
// ==============================================================================
SundialSamplerA sundialA;
SundialSamplerB sundialB;
WeatherSampler weather;
SensorGPS gps;
Adafruit_GPS GPS(&Serial1);
SensorIMU imu;

Logger logger;
Printer printer;

unsigned long breadboardLastExecutionTime = 0;
unsigned long lastSDRetryTime = 0;

// Variables to hold raw data for the Serial Monitor
float current_wind_hz = 0.0;
float current_air_v = 0.0;
float current_water_v = 0.0;

void setup() {
  // Give the IMU and other hardware 1.5 seconds to fully power up
  delay(1500);
  
  printer.init(); 

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  // Initialize E80 Sensors
  gps.init(&GPS);
  
  // --- IMU FIX 1: Initialize and force strict 100kHz I2C speed ---
  imu.init();
  Wire.setClock(100000); 

  logger.include(&gps);
  logger.include(&imu);
  logger.include(&sundialA);
  logger.include(&sundialB);
  logger.include(&weather);
  logger.init();
  
  printer.printMessage("System Init. Waiting for GPS Lock...", 10);
  breadboardLastExecutionTime = millis() - LOOP_PERIOD;
  logger.lastExecutionTime = millis() - LOOP_PERIOD;

  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countWindPulse, FALLING);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Constantly read GPS UART stream
  gps.read(&GPS);

  // --- 0. FIELD DEPLOYMENT: SD CARD AUTO-RECOVERY ---
  if (!logger.keepLogging && (currentTime - lastSDRetryTime > 3000)) {
    lastSDRetryTime = currentTime;
    printer.printMessage("Attempting to auto-recover SD Card...", 3);
    logger.init(); 
  }
    
  // --- 1. SENSOR SAMPLING LOOP ---
  if ( currentTime - breadboardLastExecutionTime > LOOP_PERIOD ) {
    breadboardLastExecutionTime = currentTime;
    
    // --- MUX (Sundial) ---
    for (int i = 0; i < 16; i++) {
      digitalWrite(MUX_S0, (i & 0x01) ? HIGH : LOW);
      digitalWrite(MUX_S1, (i & 0x02) ? HIGH : LOW);
      digitalWrite(MUX_S2, (i & 0x04) ? HIGH : LOW);
      digitalWrite(MUX_S3, (i & 0x08) ? HIGH : LOW);

      delayMicroseconds(500);
      float voltage = analogRead(MUX_SIG) * (3.3 / 4095.0);
      
      if (i < 8) sundialA.ch[i] = voltage;
      else sundialB.ch[i - 8] = voltage;
    }

    // --- Anemometer ---
    noInterrupts();
    unsigned long safePulsePeriod = pulsePeriod;
    unsigned long safeLastPulseTime = lastPulseTime;
    interrupts();
    
    if (safePulsePeriod > 0 && (currentTime - safeLastPulseTime) < 2000) {
      current_wind_hz = 1000.0 / safePulsePeriod;
      weather.wind_vel = (ANEM_SLOPE * current_wind_hz) + ANEM_INTERCEPT; 
    } else {
      current_wind_hz = 0.0;
      weather.wind_vel = 0.0; 
    }
    
    // --- Thermistors ---
    current_air_v = analogRead(THERM_AIR) * (3.3 / 4095.0);
    current_water_v = analogRead(THERM_WATER) * (3.3 / 4095.0);
    
    weather.therm_air = calculateTemperature(current_air_v);       
    weather.therm_water = calculateTemperature(current_water_v);   

    // --- Core Sensors ---
    // --- IMU FIX 2: Protect I2C communication from Interrupts ---
    noInterrupts();
    imu.read();
    interrupts();
    // gps.updateState(&GPS); // <-- Intentionally omitted to fix compile error
  }

  // --- 2. E80 PRINTER LOOP ---
  if ( currentTime - printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    
    // Format GPS Status String
    String gpsStatus = "Sats: " + String(gps.state.num_sat);
    if (gps.state.num_sat >= 4) gpsStatus += " (READY!)";
    else gpsStatus += " (WAITING)";

    // Print SD & GPS Status
    if (logger.keepLogging) printer.printValue(1, "LOGGING | " + gpsStatus);
    else printer.printValue(1, "SD ERROR | " + gpsStatus);
    
    // Print Side-by-Side Data
    printer.printValue(2, "Wind: " + String(weather.wind_vel, 2) + " m/s | " + String(current_wind_hz, 2) + " Hz");
    printer.printValue(3, "Air:  " + String(weather.therm_air, 2) + " C   | " + String(current_air_v, 2) + " V");
    printer.printValue(4, "Wat:  " + String(weather.therm_water, 2) + " C   | " + String(current_water_v, 2) + " V");
    
    printer.printValue(5, "IMU Heading: " + String(imu.state.heading, 2) + " deg");
    printer.printValue(6, "IMU MagX: " + String(imu.state.magX, 2) + " | MagY: " + String(imu.state.magY, 2));

// --- ADD SUNDIAL TO E80 PRINTER ---
    String sunA = "Sun 0-7: ";
    for(int i=0; i<8; i++) {
      sunA += String(sundialA.ch[i], 1) + " "; // 1 decimal place to save space
    }
    printer.printValue(7, sunA);

    String sunB = "Sun 8-15: ";
    for(int i=0; i<8; i++) {
      sunB += String(sundialB.ch[i], 1) + " ";
    }
    printer.printValue(8, sunB);
    // ----------------------------------

    printer.printToSerial(); 
  }

  // --- 3. E80 LOGGER LOOP ---
  if ( currentTime - logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}