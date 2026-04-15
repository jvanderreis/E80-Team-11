#ifndef __SURFACECONTROL_H__
#define __SURFACECONTROL_H__

#define SUCCESS_RADIUS 2.0 // success radius in meters

#include <Arduino.h>
#include "MotorDriver.h"
#include "XYStateEstimator.h"
extern MotorDriver motorDriver;

class SurfaceControl : public DataSource
{
public:
  SurfaceControl(void);

  void init(const int totalWayPoints_in, double * wayPoints_in, int navigateDelay_in);
  void navigate(xy_state_t * state, gps_state_t * gps_state_p, int currentTime_in);
  String printString(void);
  String printWaypointUpdate(void);
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

  // control fields
  float yaw_des;         
  float yaw;             
  float yaw_error;       
  float dist;            
  float u;               
  
  // --- PI Control Gains ---
  float Kp = 30.0;         
  float Ki = 2.5;            
  float error_integral = 0;  
  
  float Kr = 1.0;          
  float Kl = 1.0;          
  float avgPower = 50; 
  float uR;              
  float uL;              

  bool navigateState = 1;
  bool atPoint;
  bool complete = 0;
  
  // Moved to public so the main script can read it
  int currentWayPoint = 0; 

  int totalWayPoints;
  double * wayPoints;

private:
  void updatePoint(float x, float y);
  int getWayPoint(int dim);

  const int stateDims = 2;  
  bool gpsAcquired;
  int navigateDelay;
  int delayStartTime = 0;
  int currentTime;
  bool delayed;
};

#endif