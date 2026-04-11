#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

// --- E80 Framework ---
#include <Pinouts.h>
#include <TimingOffsets.h>
#include <DataSource.h>
#include <Logger.h>
#include <Printer.h>
#include <SensorGPS.h>
#include <SensorIMU.h>

// --- MUX (Sundial) Pins ---
const int MUX_S0 = 33;
const int MUX_S1 = 32;
const int MUX_S2 = 31;
const int MUX_S3 = 28;
const int MUX_SIG = A3; 

// --- Custom Sundial Data Sources ---
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

// --- Globals ---
SensorGPS gps;
Adafruit_GPS GPS(&Serial1);
SensorIMU imu;
SundialSamplerA sundialA;
SundialSamplerB sundialB;
Logger logger;
Printer printer;

unsigned long lastSampleTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  printer.init();
  
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
  
  imu.init();
  gps.init(&GPS);

  logger.include(&imu);
  logger.include(&gps);
  logger.include(&sundialA);
  logger.include(&sundialB);
  logger.init();

  // --- SD CARD FAILSAFE ---
  if (logger.keepLogging) {
    Serial.println(">>> SUCCESS: SD Card Initialized. Logging active.");
  } else {
    Serial.println(">>> ERROR: SD CARD FAILED! Check connection.");
  }

  Serial.println("System Initialized. Waiting for GPS Lock...");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 1. Keep GPS updated constantly
  gps.read(&GPS); 

  // 2. Sample Sensors & Log Data at 10Hz (every 100ms)
  if (currentTime - lastSampleTime >= 100) {
    lastSampleTime = currentTime;
    
    // Read Multiplexer
    for (int i = 0; i < 16; i++) {
      digitalWrite(MUX_S0, (i & 0x01)); 
      digitalWrite(MUX_S1, (i & 0x02));
      digitalWrite(MUX_S2, (i & 0x04)); 
      digitalWrite(MUX_S3, (i & 0x08));
      delayMicroseconds(500); // Settling time
      
      float voltage = analogRead(MUX_SIG) * (3.3 / 1023.0); 
      if (i < 8) sundialA.ch[i] = voltage;
      else sundialB.ch[i - 8] = voltage;
    }

    // Update States and Log
    imu.read();
    gps.updateState(&GPS);
    if (logger.keepLogging) {
      logger.log();
    }
  }

  // 3. Print Status to Serial Monitor every 1 second
  if (currentTime - lastPrintTime >= 1000) {
    lastPrintTime = currentTime;
    
    Serial.print("Satellites Locked: ");
    Serial.print(gps.state.num_sat);
    
    if (gps.state.num_sat >= 4) {
      Serial.println("  --> READY! You can start spinning.");
    } else {
      Serial.println("  --> Waiting for at least 4 satellites...");
    }
  }
}