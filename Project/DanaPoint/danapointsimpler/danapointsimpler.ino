#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// --- E80 PINOUT & LIBRARIES ---
#include "Pinouts.h"
#include "TimingOffsets.h"
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

// --- MISSION LOGGER ---
class MissionLogger : public DataSource {
  public:
    float state_log, shadow_calc, gps_h, gps_m, gps_s, gps_ms;
    
    MissionLogger() : DataSource(
      "mission_state,shadow_calc,gps_h,gps_m,gps_s,gps_ms", 
      "float,float,float,float,float,float") {}
    
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      data_ptr[0] = state_log; data_ptr[1] = shadow_calc;
      data_ptr[2] = gps_h;     data_ptr[3] = gps_m;
      data_ptr[4] = gps_s;     data_ptr[5] = gps_ms;
      return idx + 6 * sizeof(float);
    }
};

// --- HARDWARE CONSTANTS ---
const int MUX_S0 = 33, MUX_S1 = 32, MUX_S2 = 31, MUX_S3 = 28, MUX_SIG = A3; 
const int ANEMOMETER_PIN = A0, THERM_WATER = A1, THERM_AIR = A2;
const float BETA = 3950.0, R_NOMINAL = 100000.0, T_NOMINAL = 298.15;
const float V_SOURCE = 5.0, R_FIXED = 100000.0, GAIN = 2.7, OFFSET_CONST = 7.63125;
const float ANEM_SLOPE = 2.1587, ANEM_INTERCEPT = 1.2119;

// --- DATASOURCES ---
class SundialSamplerA : public DataSource {
  public: float ch[8];
    SundialSamplerA() : DataSource("ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7", "float,float,float,float,float,float,float,float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      for(int i = 0; i < 8; i++) data_ptr[i] = ch[i];
      return idx + 8 * sizeof(float);
    }
};
class SundialSamplerB : public DataSource {
  public: float ch[8];
    SundialSamplerB() : DataSource("ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15", "float,float,float,float,float,float,float,float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      for(int i = 0; i < 8; i++) data_ptr[i] = ch[i];
      return idx + 8 * sizeof(float);
    }
};
class WeatherSampler : public DataSource {
  public: float wind_vel, therm_air, therm_water;
    WeatherSampler() : DataSource("wind_vel,therm_air,therm_water", "float,float,float") {}
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      data_ptr[0] = wind_vel; data_ptr[1] = therm_air; data_ptr[2] = therm_water;
      return idx + 3 * sizeof(float);
    }
};

// --- GLOBALS ---
SundialSamplerA sundialA; SundialSamplerB sundialB; WeatherSampler weather;
SensorGPS gps; Adafruit_GPS GPS(&Serial1); SensorIMU imu; MotorDriver motorDriver;
SurfaceControl surfaceControl; XYStateEstimator xyEstimator; Logger logger; 
Printer printer; GPSLockLED gpsLockLED; MissionLogger missionLog;

unsigned long breadboardLastExecutionTime = 0, last_control_time = 0;
volatile unsigned long lastPulseTime_us = 0, pulsePeriod_us = 0;
float current_wind_hz = 0.0, shadow_angle_deg = 0.0; 

int mission_state = 0; 
unsigned long state_start_time = 0;

void countWindPulse() {
  unsigned long currentPulseTime_us = micros();
  if (currentPulseTime_us - lastPulseTime_us > 10000) { 
    pulsePeriod_us = currentPulseTime_us - lastPulseTime_us;
    lastPulseTime_us = currentPulseTime_us;
  }
}
float calculateTemperature(float vOut) {
  float vIn = (OFFSET_CONST - vOut) / GAIN;
  if (vIn <= 0.01 || vIn >= V_SOURCE) return -999.0;
  float rTherm = R_FIXED * ((V_SOURCE / vIn) - 1.0);
  float steinhart = log(rTherm / R_NOMINAL) / BETA; steinhart += 1.0 / T_NOMINAL;            
  return (1.0 / steinhart) - 273.15; 
}

void setup() {
  delay(1500); 
  printer.init(); Serial.begin(9600); 
  pinMode(USER_BUTTON, INPUT_PULLUP); 
  gpsLockLED.init(); motorDriver.init(); gps.init(&GPS); imu.init();
  Wire.setClock(100000);

  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP); analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countWindPulse, FALLING);

  // --- WAYPOINTS ---
  double myWaypoints[] = {
    15.0, 0.0,    // WP 0: Transit out (Ignored in State 1, but keeps array intact)
    2.0,  0.0     // WP 1: Drive back to the dock
  };
  surfaceControl.init(2, myWaypoints, 0);

  logger.include(&gps); logger.include(&imu); logger.include(&motorDriver);
  logger.include(&surfaceControl); logger.include(&sundialA); logger.include(&sundialB);
  logger.include(&weather); logger.include(&missionLog); logger.include(&xyEstimator); 
  logger.init();

  Serial.println("\n*** WAITING FOR BUTTON PRESS ***");
  while (digitalRead(USER_BUTTON) == HIGH) { 
    gps.read(&GPS); 
    if (millis() % 100 < 50) gpsLockLED.flashLED(&gps.state);
  }

  Serial.println("10-SEC COUNTDOWN!");
  for (int i = 10; i > 0; i--) { digitalWrite(GPS_LOCK_LED, HIGH); delay(500); digitalWrite(GPS_LOCK_LED, LOW); delay(500); }

  mission_state = 1; 
  state_start_time = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long activePeriod = logger.keepLogging ? LOOP_PERIOD : 100; 
  gps.read(&GPS);

  static unsigned long lastSDRetryTime = 0;
  if (!logger.keepLogging && (currentTime - lastSDRetryTime > 3000)) {
    lastSDRetryTime = currentTime;
    logger.init(); 
  }

  // --- 1. SENSOR SAMPLING ---
  if ( currentTime - breadboardLastExecutionTime > activePeriod ) {
    breadboardLastExecutionTime = currentTime;
    
    float max_voltage = 0;
    for (int i = 0; i < 16; i++) {
      digitalWrite(MUX_S0, (i & 0x01) ? HIGH : LOW); digitalWrite(MUX_S1, (i & 0x02) ? HIGH : LOW);
      digitalWrite(MUX_S2, (i & 0x04) ? HIGH : LOW); digitalWrite(MUX_S3, (i & 0x08) ? HIGH : LOW);
      delayMicroseconds(500);
      float voltage = analogRead(MUX_SIG) * (3.3 / 4095.0);
      if (i < 8) sundialA.ch[i] = voltage; else sundialB.ch[i - 8] = voltage;
      if (voltage > max_voltage) max_voltage = voltage;
    }

    float X_comp = 0, Y_comp = 0;
    for (int i = 0; i < 16; i++) {
      float v = (i < 8) ? sundialA.ch[i] : sundialB.ch[i - 8];
      float shadow_val = max_voltage - v; float angle_rad = i * (PI / 8.0);
      X_comp += shadow_val * cos(angle_rad); Y_comp += shadow_val * sin(angle_rad);
    }
    
    float shadow_angle_rad = atan2(Y_comp, X_comp);
    if (shadow_angle_rad < 0) shadow_angle_rad += 2 * PI;
    shadow_angle_deg = fmod(360.0 - (shadow_angle_rad * (180.0 / PI)), 360.0);

    noInterrupts(); unsigned long safePulsePeriod = pulsePeriod_us; unsigned long safeLastPulse = lastPulseTime_us; interrupts();
    if (safePulsePeriod > 0 && (micros() - safeLastPulse) < 2000000) {
      weather.wind_vel = (ANEM_SLOPE * (1000000.0 / safePulsePeriod)) + ANEM_INTERCEPT; 
    } else weather.wind_vel = 0.0; 
    
    // Software filter for Thermistors (Reduced to 3.3 multiplier)
    float sum_air = 0, sum_water = 0;
    for(int i=0; i<10; i++) { sum_air += analogRead(THERM_AIR); sum_water += analogRead(THERM_WATER); delayMicroseconds(100); }
    weather.therm_air = calculateTemperature((sum_air / 10.0) * (3.3 / 4095.0));       
    weather.therm_water = calculateTemperature((sum_water / 10.0) * (3.3 / 4095.0));   
    imu.read(); xyEstimator.updateState(&imu.state, &gps.state);
  }

  // --- 2. GNC MISSION STATE MACHINE ---
  if ( currentTime - last_control_time > activePeriod && mission_state > 0) {
    last_control_time = currentTime;
    
    // STATE 1: Dumb Transit (20 Seconds)
    if (mission_state == 1) {
      if (millis() - state_start_time < 20000) {
        motorDriver.drive(80, 80, 0); 
      } else {
        motorDriver.drive(0, 0, 0); delay(100); // Voltage stabilize
        mission_state = 2; state_start_time = millis();
      }
    }
    // STATE 2: Spin Calibration (20 Seconds)
    else if (mission_state == 2) {
      if (millis() - state_start_time < 20000) {
        motorDriver.drive(60, -60, 0); 
      } else {
        motorDriver.drive(0, 0, 0); delay(100);
        mission_state = 3; state_start_time = millis();
      }
    }
    // STATE 3: Silent Pause (20 Seconds)
    else if (mission_state == 3) {
      if (millis() - state_start_time < 20000) {
        motorDriver.drive(0, 0, 0); 
      } else {
        mission_state = 4; state_start_time = millis();
      }
    }
    // STATE 4: Sun Dash (40 Seconds)
    else if (mission_state == 4) {
      if (millis() - state_start_time < 40000) {
        float error = 180.0 - shadow_angle_deg;
        while (error <= -180.0) error += 360.0;
        while (error >   180.0) error -= 360.0;
        
        int turn_effort = 0.3 * error; 
        turn_effort = constrain(turn_effort, -40, 40); 
        
        // The Deadband: If within 20 deg, don't wiggle!
        if (abs(error) < 20.0) motorDriver.drive(60, 60, 0);
        else motorDriver.drive(60 + turn_effort, 60 - turn_effort, 0); 

      } else {
        motorDriver.drive(0, 0, 0); delay(100);
        mission_state = 5; 
        
        // CRITICAL FIX: Tell the library we are going home!
        surfaceControl.currentWayPoint = 1; 
      }
    }
    // STATE 5: Return Home
    else if (mission_state == 5) {
      surfaceControl.navigate(&xyEstimator.state, &gps.state, currentTime);
      motorDriver.drive((int)surfaceControl.uL, (int)surfaceControl.uR, 0);
      if (surfaceControl.complete) {
        motorDriver.drive(0,0,0);
        mission_state = 0; 
      }
    }
  }

  // --- 3. LOGGING ---
  if ( currentTime - logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    missionLog.state_log = (float)mission_state; 
    missionLog.shadow_calc = shadow_angle_deg;
    missionLog.gps_h = GPS.hour; missionLog.gps_m = GPS.minute;
    missionLog.gps_s = GPS.seconds; missionLog.gps_ms = GPS.milliseconds;
    logger.lastExecutionTime = currentTime; logger.log();
  }
}