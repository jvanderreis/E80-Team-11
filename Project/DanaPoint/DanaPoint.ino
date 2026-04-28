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

// --- EXPANDED MISSION LOGGER ---
class MissionLogger : public DataSource {
  public:
    float state_log, shadow_calc, gps_h, gps_m, gps_s, gps_ms;
    float target_ang, p_effort, d_effort; // --- NEW: PD tracking vars ---
    
    MissionLogger() : DataSource(
      "mission_state,shadow_calc,gps_h,gps_m,gps_s,gps_ms,target_ang,p_effort,d_effort", 
      "float,float,float,float,float,float,float,float,float") {}
    
    size_t writeDataBytes(unsigned char * buffer, size_t idx) override {
      float * data_ptr = (float *)(buffer + idx);
      data_ptr[0] = state_log; data_ptr[1] = shadow_calc;
      data_ptr[2] = gps_h;     data_ptr[3] = gps_m;
      data_ptr[4] = gps_s;     data_ptr[5] = gps_ms;
      data_ptr[6] = target_ang;data_ptr[7] = p_effort; // --- NEW ---
      data_ptr[8] = d_effort;                          // --- NEW ---
      return idx + 9 * sizeof(float);
    }
};

// --- HARDWARE CONSTANTS ---
const int MUX_S0 = 33, MUX_S1 = 32, MUX_S2 = 31, MUX_S3 = 28, MUX_SIG = A3; 
const int ANEMOMETER_PIN = A0, THERM_WATER = A1, THERM_AIR = A2;
const float BETA = 3950.0, R_NOMINAL = 100000.0, T_NOMINAL = 298.15;
const float V_SOURCE = 5.0, R_FIXED = 100000.0, GAIN = 2.7, OFFSET_CONST = 7.63125;
const float ANEM_SLOPE = 2.1587, ANEM_INTERCEPT = 1.2119;

// --- DATASOURCES (Same as your original code) ---
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

// --- MISSION STATE VARIABLES ---
int mission_state = 0; // 0=Wait, 1=Outbound, 2=Sun Hold, 3=IMU Hold, 4=Drift, 5=Return
unsigned long state_start_time = 0;

// --- UNIFIED PD CONTROL HELPER ---
float Kp_heading = 1.5; 
float Kd_heading = 0.8; 
float global_last_error = 0.0;

// --- NEW: Globals to pass data to the logger ---
float log_target_angle = 0.0;
float log_p_term = 0.0;
float log_d_term = 0.0;

int calculateMotorEffort(float target_angle, float current_angle) {
    float error = target_angle - current_angle;
    while (error <= -180.0) error += 360.0;
    while (error >   180.0) error -= 360.0;
    
    float p_term = Kp_heading * error;
    float d_term = Kd_heading * (error - global_last_error);
    global_last_error = error; 
    
    // --- NEW: Save to globals for the MissionLogger ---
    log_target_angle = target_angle;
    log_p_term = p_term;
    log_d_term = d_term;
    
    int turn_speed = p_term + d_term;
    return constrain(turn_speed, -80, 80); 
}

// --- INTERRUPTS & MATH ---
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

  // --- DANA POINT WAYPOINTS (Local Coordinates) ---
double myWaypoints[] = {
    20.0, 0.0,    // WP 1: Drive 20m straight Forward out to open water
    20.0, 0.0,    // WP 2: Return to the safe point (Clears the drift zone)
    2.0,  0.0     // WP 3: Drive straight back Forward toward you
  };
surfaceControl.init(3, myWaypoints, 0);

  logger.include(&gps); logger.include(&imu); logger.include(&motorDriver);
  logger.include(&surfaceControl); logger.include(&sundialA); logger.include(&sundialB);
  logger.include(&weather); logger.include(&missionLog); logger.include(&xyEstimator); logger.init();

  Serial.println("\n*** WAITING FOR PHYSICAL BUTTON PRESS ***");
  while (digitalRead(USER_BUTTON) == HIGH) { 
    gps.read(&GPS); 
    if (millis() % 100 < 50) gpsLockLED.flashLED(&gps.state);
  }

  Serial.println("BUTTON PRESSED! 10-SEC COUNTDOWN TO MISSION START!");
  for (int i = 10; i > 0; i--) { digitalWrite(GPS_LOCK_LED, HIGH); delay(500); digitalWrite(GPS_LOCK_LED, LOW); delay(500); }

  mission_state = 1; // Start Transit
  state_start_time = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long activePeriod = logger.keepLogging ? LOOP_PERIOD : 100; 
  gps.read(&GPS);

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
    
    weather.therm_air = calculateTemperature(analogRead(THERM_AIR) * (3.3 / 4095.0));       
    weather.therm_water = calculateTemperature(analogRead(THERM_WATER) * (3.3 / 4095.0));   
    imu.read(); xyEstimator.updateState(&imu.state, &gps.state);
  }

  // --- 2. GNC MISSION STATE MACHINE ---
  if ( currentTime - last_control_time > activePeriod && mission_state > 0) {
    last_control_time = currentTime;
    
    // STATE 1: Outbound Transit to WP 1
    if (mission_state == 1) {
      surfaceControl.navigate(&xyEstimator.state, &gps.state, currentTime);
      motorDriver.drive((int)surfaceControl.uL, (int)surfaceControl.uR, 0);
      
      if (surfaceControl.currentWayPoint == 1) { // Hit WP 1!
        mission_state = 2;
        state_start_time = millis();
        global_last_error = 0; // Reset PD memory
      }
    }
    // STATE 2: Sundial PD Hold (45 Seconds)
    else if (mission_state == 2) {
      if (millis() - state_start_time < 45000) {
        int effort = calculateMotorEffort(90.0, shadow_angle_deg); // Target Sun at 90 deg relative
        motorDriver.drive(-effort, effort, 0); 
      } else {
        mission_state = 3;
        state_start_time = millis();
        global_last_error = 0; 
      }
    }
    // STATE 3: IMU PD Hold (45 Seconds)
    else if (mission_state == 3) {
      static float target_imu_heading = imu.state.heading + 90.0; // Flip 90 deg and lock
      if (target_imu_heading >= 360.0) target_imu_heading -= 360.0;

      if (millis() - state_start_time < 45000) {
        int effort = calculateMotorEffort(target_imu_heading, imu.state.heading); 
        motorDriver.drive(effort, -effort, 0); // Note: IMU yaw logic is inverted from Sundial yaw
      } else {
        mission_state = 4;
        state_start_time = millis();
      }
    }
    // STATE 4: Silent Drift / Thermal Baseline (60 Seconds)
    else if (mission_state == 4) {
      if (millis() - state_start_time < 60000) {
        motorDriver.drive(0, 0, 0); // MOTORS OFF
      } else {
        mission_state = 5;
      }
    }
    // STATE 5: Return to Dock (WP 2)
    else if (mission_state == 5) {
      surfaceControl.navigate(&xyEstimator.state, &gps.state, currentTime);
      motorDriver.drive((int)surfaceControl.uL, (int)surfaceControl.uR, 0);
      if (surfaceControl.complete) {
        motorDriver.drive(0,0,0);
        mission_state = 0; // Mission Finished
      }
    }
  }

// LOGGING
  if ( currentTime - logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    
    missionLog.state_log = (float)mission_state; 
    missionLog.shadow_calc = shadow_angle_deg;
    missionLog.gps_h = GPS.hour;
    missionLog.gps_m = GPS.minute;
    missionLog.gps_s = GPS.seconds;
    missionLog.gps_ms = GPS.milliseconds;
    
    missionLog.target_ang = log_target_angle;
    missionLog.p_effort = log_p_term;
    missionLog.d_effort = log_d_term;

    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}