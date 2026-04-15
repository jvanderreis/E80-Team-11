#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// --- E80 PINOUT & TIMING DEFINITIONS ---
#include "Pinouts.h"
#include "TimingOffsets.h"

// --- E80 FRAMEWORK LIBRARIES ---
#include "DataSource.h"
#include "Logger.h"
#include "Printer.h"
#include "SensorGPS.h"
#include "SensorIMU.h"
#include "MotorDriver.h"
#include "SurfaceControl.h"
#include "XYStateEstimator.h"
#include <Adafruit_GPS.h>
#include "GPSLockLED.h"

// ==============================================================================
// 1. HARDWARE PINS & CONSTANTS
// ==============================================================================
const int MUX_S0 = 33;
const int MUX_S1 = 32;
const int MUX_S2 = 31;
const int MUX_S3 = 28;
const int MUX_SIG = A3; 

const int ANEMOMETER_PIN = A0;
const int THERM_WATER = A1;
const int THERM_AIR = A2;

const float BETA = 3950.0;
const float R_NOMINAL = 100000.0; 
const float T_NOMINAL = 298.15;
const float V_SOURCE = 5.0;       
const float R_FIXED = 100000.0;
const float GAIN = 2.7;           
const float OFFSET_CONST = 7.63125;

const float ANEM_SLOPE = 2.1587;
const float ANEM_INTERCEPT = 1.2119;

int DEPLOYMENT_STATE = -1;

// ==============================================================================
// 2. INTERRUPTS & MATH HELPERS
// ==============================================================================
volatile unsigned long lastPulseTime_us = 0;
volatile unsigned long pulsePeriod_us = 0;

void countWindPulse() {
  unsigned long currentPulseTime_us = micros();
  if (currentPulseTime_us - lastPulseTime_us > 10000) { // 10ms debounce
    pulsePeriod_us = currentPulseTime_us - lastPulseTime_us;
    lastPulseTime_us = currentPulseTime_us;
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
// 3. DATASOURCE CLASSES
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
// 4. GLOBALS
// ==============================================================================
SundialSamplerA sundialA;
SundialSamplerB sundialB;
WeatherSampler weather;
SensorGPS gps;
Adafruit_GPS GPS(&Serial1);
SensorIMU imu;
MotorDriver motorDriver;
SurfaceControl surfaceControl;
XYStateEstimator xyEstimator;
Logger logger;
Printer printer;
GPSLockLED gpsLockLED;

unsigned long breadboardLastExecutionTime = 0;
unsigned long last_control_time = 0;
unsigned long lastSDRetryTime = 0;

float current_wind_hz = 0.0;
float current_air_v = 0.0;
float current_water_v = 0.0;
float shadow_angle_deg = 0.0; // Live calculated sundial angle

// ==============================================================================
// SETUP
// ==============================================================================

void setup() {
  delay(1500); // CRITICAL: Gives IMU time to power up on battery
  
  printer.init(); 
  Serial.begin(9600); // Open Serial for your prompt
  
  // Initialize E80 Hardware
  pinMode(USER_BUTTON, INPUT_PULLUP); // Pin 2 on Motherboard
  gpsLockLED.init();
  motorDriver.init();
  gps.init(&GPS);
  imu.init();
  Wire.setClock(100000);

  // Hardware Pinmodes
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countWindPulse, FALLING);


  double myWaypoints[] = {
    0.0, -15.0,   // WP 1: Drive 15m South into open water
    10.0, -15.0,  // WP 2: Drive 10m East for a thermal transect
    2.0, -2.0     // WP 3: Return to the dock (stopping 2m short to avoid a crash!)
  }; 
  int numWaypoints = 3; 
  surfaceControl.init(numWaypoints, myWaypoints, 0);

  // Setup Logger
  logger.include(&gps);
  logger.include(&imu);
  logger.include(&motorDriver);
  logger.include(&surfaceControl);
  logger.include(&sundialA);
  logger.include(&sundialB);
  logger.include(&weather);
  logger.init();

  // =======================================================
  // ROBUST DEPLOYMENT ARMING SEQUENCE
  // =======================================================
  
  Serial.println("\n=====================================");
  Serial.println("  ROBOT READY. WAITING FOR COMMAND.");
  Serial.println("  TYPE A NUMBER IN SERIAL MONITOR:");
  Serial.println("  1 = CALIBRATION SPIN (Phase 1)");
  Serial.println("  2 = SEEK THE SUN (Phase 2)");
  Serial.println("  3 = GPS WAYPOINTS (Phase 3)");
  Serial.println("=====================================\n");

// STAGE 1: Wait for Serial Input
  unsigned long lastLEDUpdate = 0;
  while (DEPLOYMENT_STATE == -1) {
    if (Serial.available() > 0) {
      char incomingByte = Serial.read();
      if (incomingByte == '1') DEPLOYMENT_STATE = 1;
      else if (incomingByte == '2') DEPLOYMENT_STATE = 2;
      else if (incomingByte == '3') DEPLOYMENT_STATE = 3;
    }
    
    // 1. Keep reading GPS as FAST as possible (no delays!)
    gps.read(&GPS);
    
    // 2. Only update the LED animation every 50ms
    if (millis() - lastLEDUpdate > 50) {
      gpsLockLED.flashLED(&gps.state); 
      lastLEDUpdate = millis();
    }
  }

  Serial.print("SUCCESS: Mode "); 
  Serial.print(DEPLOYMENT_STATE); 
  Serial.println(" selected!");
  Serial.println("\n*** STAGE 2: DISCONNECT LAPTOP NOW ***");
  Serial.println("PRESS THE PHYSICAL 'USER BUTTON' ON THE MOTHERBOARD TO START 10-SEC COUNTDOWN.");

  // STAGE 2: Wait for Physical Button Press
  while (digitalRead(USER_BUTTON) == HIGH) { 
    gps.read(&GPS); // Read as fast as possible
    
    if (millis() - lastLEDUpdate > 50) {
      gpsLockLED.flashLED(&gps.state);
      lastLEDUpdate = millis();
    }
  }

  // STAGE 3: Seal the Box Countdown
  Serial.println("BUTTON PRESSED! MOTORS ARMING IN 10 SECONDS! SEAL THE TUPPERWARE!");
  for (int i = 10; i > 0; i--) {
    Serial.print(i); Serial.println("...");
    
    // Flash LED rapidly to warn you it's about to move
    digitalWrite(GPS_LOCK_LED, HIGH);
    delay(500);
    digitalWrite(GPS_LOCK_LED, LOW);
    delay(500);
  }

  // Set timing variables so the main loop doesn't trigger a massive initial control jump
  breadboardLastExecutionTime = millis();
  last_control_time = millis();
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  unsigned long currentTime = millis();
  unsigned long activePeriod = logger.keepLogging ? LOOP_PERIOD : 100; 

  gps.read(&GPS);

  // SD Auto-Recovery
  if (!logger.keepLogging && (currentTime - lastSDRetryTime > 3000)) {
    lastSDRetryTime = currentTime;
    logger.init(); 
  }

  // --- 1. SENSOR SAMPLING LOOP ---
  if ( currentTime - breadboardLastExecutionTime > activePeriod ) {
    breadboardLastExecutionTime = currentTime;
    
    // Read MUX
    float max_voltage = 0;
    for (int i = 0; i < 16; i++) {
      digitalWrite(MUX_S0, (i & 0x01) ? HIGH : LOW);
      digitalWrite(MUX_S1, (i & 0x02) ? HIGH : LOW);
      digitalWrite(MUX_S2, (i & 0x04) ? HIGH : LOW);
      digitalWrite(MUX_S3, (i & 0x08) ? HIGH : LOW);

      delayMicroseconds(500);
      float voltage = analogRead(MUX_SIG) * (3.3 / 4095.0);
      
      if (i < 8) sundialA.ch[i] = voltage;
      else sundialB.ch[i - 8] = voltage;
      
      if (voltage > max_voltage) max_voltage = voltage;
    }

    // Live C++ calculation of Shadow Angle (Matches MATLAB logic)
    float X_comp = 0;
    float Y_comp = 0;
    for (int i = 0; i < 16; i++) {
      float v = (i < 8) ? sundialA.ch[i] : sundialB.ch[i - 8];
      float shadow_val = max_voltage - v;
      float angle_rad = i * (PI / 8.0); // 22.5 deg per sensor
      X_comp += shadow_val * cos(angle_rad);
      Y_comp += shadow_val * sin(angle_rad);
    }
    
    // SENSOR_DIR (-1 for Flipped, 1 for Normal) applied below
    float shadow_angle_rad = atan2(Y_comp, X_comp);
    if (shadow_angle_rad < 0) shadow_angle_rad += 2 * PI;
    shadow_angle_deg = 360.0 - (shadow_angle_rad * (180.0 / PI)); // Flipped direction math
    shadow_angle_deg = fmod(shadow_angle_deg, 360.0);

    // Read Anemometer
    noInterrupts();
    unsigned long safePulsePeriod_us = pulsePeriod_us;
    unsigned long safeLastPulseTime_us = lastPulseTime_us;
    interrupts();
    
    if (safePulsePeriod_us > 0 && (micros() - safeLastPulseTime_us) < 2000000) {
      current_wind_hz = 1000000.0 / safePulsePeriod_us; 
      weather.wind_vel = (ANEM_SLOPE * current_wind_hz) + ANEM_INTERCEPT; 
    } else {
      current_wind_hz = 0.0;
      weather.wind_vel = 0.0; 
    }
    
    // Read Thermistors
    current_air_v = analogRead(THERM_AIR) * (3.3 / 4095.0);
    current_water_v = analogRead(THERM_WATER) * (3.3 / 4095.0);
    weather.therm_air = calculateTemperature(current_air_v);       
    weather.therm_water = calculateTemperature(current_water_v);   

    // Read IMU & Update State
    imu.read();
    xyEstimator.updateState(&imu.state, &gps.state);
  }

// --- 2. MOTOR CONTROL LOOP ---
  if ( currentTime - last_control_time > activePeriod ) {
    last_control_time = currentTime;

    if (DEPLOYMENT_STATE == 0) {
      motorDriver.drive(0, 0, 0); 

    } else if (DEPLOYMENT_STATE == 1) {
      // PHASE 1: SPIN CALIBRATION
      motorDriver.drive(80, -80, 0); 

    } else if (DEPLOYMENT_STATE == 2) {
      // PHASE 2: HYBRID SEEKING (60s SUNDIAL -> 90 DEGREE FLIP -> IMU HOLD)
      
      // Static variables keep their value between loop runs
      static unsigned long phase2_start_time = millis();
      static bool imu_mode_active = false;
      static float target_imu_heading = 0.0;

      // If this is the very first time we enter State 2, reset the timer
      static bool first_run = true;
      if (first_run) {
        phase2_start_time = millis();
        first_run = false;
      }

      // --- PART A: SUNDIAL SEEKING (FIRST 60 SECONDS) ---
      if (!imu_mode_active) {
        
        if (millis() - phase2_start_time < 60000) {
          // Standard 10-Degree Deadstop Sundial Logic
          float sun_offset_deg = 0.0; 
          float target_shadow_rad = (180.0 + sun_offset_deg) * (PI / 180.0); 
          float current_shadow_rad = shadow_angle_deg * (PI / 180.0);
          
          float yaw_error = target_shadow_rad - current_shadow_rad;
          while (yaw_error < -PI) yaw_error += 2*PI;
          while (yaw_error >  PI) yaw_error -= 2*PI;
          
          float error_deg = abs(yaw_error) * (180.0 / PI);

          if (error_deg <= 10.0) {
            motorDriver.drive(0, 0, 0); // 10-DEGREE PAUSE
          } else {
            int turn_speed = map((long)error_deg, 10, 180, 35, 100);
            turn_speed = constrain(turn_speed, 35, 100);
            
            if (yaw_error > 0) motorDriver.drive(-turn_speed, turn_speed, 0);
            else               motorDriver.drive(turn_speed, -turn_speed, 0);
          }
        } 
        else {
          // 60 SECONDS ARE UP: Transition to IMU mode
          imu_mode_active = true;
          // Set target to current heading + 90 degrees (flip to the side)
          target_imu_heading = imu.state.heading + 90.0;
          // Wrap heading to stay between 0 and 360
          if (target_imu_heading >= 360.0) target_imu_heading -= 360.0;
        }
      }

      // --- PART B: IMU HEADING HOLD (AFTER 60 SECONDS) ---
      if (imu_mode_active) {
        float current_heading = imu.state.heading;
        float heading_error = target_imu_heading - current_heading;

        // Wrap error to strictly -180 to +180 degrees
        while (heading_error <= -180.0) heading_error += 360.0;
        while (heading_error >   180.0) heading_error -= 360.0;

        float error_mag = abs(heading_error);

        // Optional 5-degree deadband for the IMU so it doesn't jitter endlessly
        if (error_mag <= 5.0) {
           motorDriver.drive(0, 0, 0);
        } else {
           // Map error (5 to 180 degrees) to motor speeds (35 to 80)
           int turn_speed = map((long)error_mag, 5, 180, 35, 80);
           turn_speed = constrain(turn_speed, 35, 80);

           // Positive error means target is to the right (increase heading)
           if (heading_error > 0) motorDriver.drive(turn_speed, -turn_speed, 0); // Turn Right
           else                   motorDriver.drive(-turn_speed, turn_speed, 0); // Turn Left
        }
      }

    } else if (DEPLOYMENT_STATE == 3) {
      // PHASE 3: WAYPOINTS
      surfaceControl.navigate(&xyEstimator.state, &gps.state, currentTime);
      motorDriver.drive((int)surfaceControl.uL, (int)surfaceControl.uR, 0);
    }
  }

  // --- 3. E80 PRINTER & LOGGER ---
  if ( currentTime - printer.lastExecutionTime > activePeriod ) {
    printer.lastExecutionTime = currentTime;
    
    String gpsStatus = "Sats: " + String(gps.state.num_sat);
    if (gps.state.num_sat >= 4) {
      gpsStatus += " (READY!)";
    } else {
      gpsStatus += " (WAITING)";
    }

    if (logger.keepLogging) printer.printValue(1, "LOGGING | " + gpsStatus);
    else printer.printValue(1, "SD ERROR | " + gpsStatus);
    
    printer.printValue(2, "Motors L:" + String(motorDriver.motorValues[0]) + " R:" + String(motorDriver.motorValues[1]));
    printer.printValue(3, "Wind: " + String(weather.wind_vel, 2) + " m/s | " + String(current_wind_hz, 2) + " Hz");
    printer.printValue(4, "Air:  " + String(weather.therm_air, 2) + " C | Wat: " + String(weather.therm_water, 2) + " C");
    printer.printValue(5, "IMU Heading: " + String(imu.state.heading, 2) + " deg");
    printer.printValue(6, "Live Shadow Angle: " + String(shadow_angle_deg, 2) + " deg");

    String sunA = "Sun 0-7: ";
    for(int i=0; i<8; i++) sunA += String(sundialA.ch[i], 1) + " "; 
    printer.printValue(7, sunA);

    String sunB = "Sun 8-15: ";
    for(int i=0; i<8; i++) sunB += String(sundialB.ch[i], 1) + " ";
    printer.printValue(8, sunB);

    printer.printToSerial(); 
  }

  if ( currentTime - logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}