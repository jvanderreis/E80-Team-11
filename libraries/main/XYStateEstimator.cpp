#include "XYStateEstimator.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

XYStateEstimator::XYStateEstimator(void)
  : DataSource("x,y","float,float") // from DataSource
{}

void XYStateEstimator::init(void) {
 	state.x = 0;
  state.y = 0;
  state.yaw = 0;
}

void XYStateEstimator::updateState(imu_state_t * imu_state_p, gps_state_t * gps_state_p) {
  if (gps_state_p->num_sat >= N_SATS_THRESHOLD){
    gpsAcquired = 1;

    // Convert all degree measurements to radians for the math functions
    float lat_rad = gps_state_p->lat * (PI / 180.0);
    float lon_rad = gps_state_p->lon * (PI / 180.0);
    float origin_lat_rad = origin_lat * (PI / 180.0);
    float origin_lon_rad = origin_lon * (PI / 180.0);

    // Forward Equirectangular Projection to get X and Y in meters
    // state.y is the North/South distance from the origin
    state.y = RADIUS_OF_EARTH * (lat_rad - origin_lat_rad);
    
    // state.x is the East/West distance (scaled by the cosine of the origin latitude)
    state.x = RADIUS_OF_EARTH * (lon_rad - origin_lon_rad) * cos(origin_lat_rad);

    // Convert compass heading to standard math yaw
    // Compass: 0 is North, increases Clockwise.
    // Math (ENU): 0 is East, increases Counter-Clockwise.
    float heading_rad = imu_state_p->heading * (PI / 180.0);
    state.yaw = (PI / 2.0) - heading_rad;

    // Bound the yaw angle between -PI and PI 
    if (state.yaw > PI) {
      state.yaw -= 2.0 * PI;
    } else if (state.yaw <= -PI) {
      state.yaw += 2.0 * PI;
    }

  }
  else{
    gpsAcquired = 0;
  }
}

String XYStateEstimator::printState(void) {
  String currentState = "";
  int decimals = 2;
  if (!gpsAcquired){
    currentState += "XY_State: Waiting to acquire more satellites...";
  }
  else{
    currentState += "XY_State: x: ";
    currentState += String(state.x,decimals);
    currentState += "[m], ";
    currentState += "y: ";
    currentState += String(state.y,decimals);
    currentState += "[m], ";
    currentState += "yaw: ";
    currentState += String(state.yaw,decimals);
    currentState += "[rad]; ";
  }
  return currentState;
}

size_t XYStateEstimator::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  return idx + 2*sizeof(float);
}
