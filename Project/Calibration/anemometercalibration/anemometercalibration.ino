#include <Arduino.h>
#include <Wire.h>
#include <Pinouts.h>
#include <TimingOffsets.h>
#include <DataSource.h>
#include <Logger.h>
#include <Printer.h>

const int ANEMOMETER_PIN = A0; // Update if plugged into a different pin!

// --- Custom Data Source for Anemometer ---
class AnemometerSampler : public DataSource {
  public:
    float windHz;
    AnemometerSampler() : DataSource("windHz", "float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      data_ptr[0] = windHz;
      return idx + sizeof(float);
    }
};

AnemometerSampler windSampler;
Logger logger;
Printer printer;

// Interrupt Variables (Using micros() for much higher precision)
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulsePeriod = 0;

void countWindPulse() {
  unsigned long now = micros();
  unsigned long period = now - lastPulseTime;
  
  if (period > 5000) { // 5000 microseconds = 5ms debounce
    pulsePeriod = period;
    lastPulseTime = now;
  }
}

unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  printer.init();
  
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countWindPulse, FALLING);

  logger.include(&windSampler);
  logger.init();

  // --- SD CARD FAILSAFE ---
  if (logger.keepLogging) {
    Serial.println(">>> SUCCESS: SD Card Initialized. Logging active.");
  } else {
    Serial.println(">>> ERROR: SD CARD FAILED! Check connection.");
  }

  Serial.println("Anemometer Logger Ready. Start the wind tunnel!");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSampleTime >= 100) { // 10Hz sampling
    lastSampleTime = currentTime;
    
    // Safely grab the volatile variables
    noInterrupts();
    unsigned long currentPeriod = pulsePeriod;
    unsigned long timeSinceLast = micros() - lastPulseTime;
    interrupts();

    // Calculate Frequency safely
    if (timeSinceLast > 2000000) { 
      // If 2 seconds have passed with no pulses, the fan is stopped.
      windSampler.windHz = 0.0;
    } else if (currentPeriod > 0) {
      // 1,000,000 microseconds in a second
      windSampler.windHz = 1000000.0 / (float)currentPeriod; 
    } else {
      windSampler.windHz = 0.0;
    }

    // Log to SD Card
    if (logger.keepLogging) {
      logger.log();
    }

    // Print to Serial Monitor
    Serial.print("Anemometer Freq: ");
    Serial.print(windSampler.windHz);
    Serial.println(" Hz");
  }
}