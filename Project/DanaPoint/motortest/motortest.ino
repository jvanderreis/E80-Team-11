#include <Arduino.h>
#include "Pinouts.h"
#include "MotorDriver.h"

MotorDriver motorDriver;

void setup() {
  Serial.begin(9600);
  motorDriver.init();
  
  Serial.println("Motor Diagnostic Initializing in 3 seconds...");
  delay(3000); 
}

void loop() {
  // The E80 MotorDriver max PWM is 127
  int pwm_25 = 32;
  int pwm_50 = 64;
  int pwm_75 = 95;
  int pwm_100 = 127;
  int hold_time = 2000; // 2 seconds per step

  Serial.println("--- STARTING RAMP SEQUENCE ---");

  Serial.println("25% Power");
  motorDriver.drive(pwm_25, pwm_25, 0);
  delay(hold_time);

  Serial.println("50% Power");
  motorDriver.drive(pwm_50, pwm_50, 0);
  delay(hold_time);

  Serial.println("75% Power");
  motorDriver.drive(pwm_75, pwm_75, 0);
  delay(hold_time);

  Serial.println("100% Power");
  motorDriver.drive(pwm_100, pwm_100, 0);
  delay(hold_time);

  Serial.println("0% Power (COAST)");
  motorDriver.drive(0, 0, 0);
  delay(3000);

  Serial.println("Reverse 25% Power");
  motorDriver.drive(-pwm_25, -pwm_25, 0);
  delay(hold_time);
  
  Serial.println("0% Power (COAST)");
  motorDriver.drive(0, 0, 0);
  
  Serial.println("Sequence Complete. Restarting in 5 seconds...\n");
  delay(5000);
}