#include "SurfaceControl.h"
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

SurfaceControl::SurfaceControl(void) 
: DataSource("u,uL,uR,yaw,yaw_des,error_integral","float,float,float,float,float,float"){}

void SurfaceControl::init(const int totalWayPoints_in, double * wayPoints_in, int navigateDelay_in) {
  totalWayPoints = totalWayPoints_in;
  wayPoints = new double[2*totalWayPoints]; 
  for (int i=0; i<totalWayPoints; i++) { 
    wayPoints[i] = wayPoints_in[i];
  }
  navigateDelay = navigateDelay_in;
  if (totalWayPoints == 0) atPoint = 1; 
  else atPoint = 0; 
}

int SurfaceControl::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}

void SurfaceControl::navigate(xy_state_t * state, gps_state_t * gps_state_p, int currentTime_in) {
  currentTime = currentTime_in;

  if (gps_state_p->num_sat >= 6) { // Bumped to 6 to mitigate ocean multipath error
    gpsAcquired = 1;

    updatePoint(state->x, state->y);
    if (currentWayPoint == totalWayPoints) return; 
    
    if (atPoint || delayed) {
      uL = 0; 
      uR = 0;
      return; 
    }

    int x_des = getWayPoint(0);
    int y_des = getWayPoint(1);

    yaw = state->yaw;
    yaw_des = atan2(y_des - state->y, x_des - state->x);
    yaw_error = angleDiff(yaw_des - yaw);

    // Anti-windup
    if (abs(yaw_error) < 0.8) { 
      error_integral += yaw_error; 
    } else {
      error_integral = 0; 
    }

    // PI Control math
    u = (Kp * yaw_error) + (Ki * error_integral);

    uR = avgPower + u;
    uL = avgPower - u;

    uR = uR * Kr;
    uL = uL * Kl;

    if (uR > 127) uR = 127;
    if (uR < 0) uR = 0;
    if (uL > 127) uL = 127;
    if (uL < 0) uL = 0;
  }
  else {
    gpsAcquired = 0;
  }
}

String SurfaceControl::printString(void) {
  String printString = "";
  if (!navigateState) {
    printString += "SurfaceControl: Not in navigate state";
  }
  else if (!gpsAcquired) {
    printString += "SurfaceControl: Waiting for 6+ sats...";
  }
  else {
    printString += "SurfaceControl: Yaw_Des: " + String(yaw_des*180.0/PI) + "[deg], ";
    printString += "Yaw: " + String(yaw*180.0/PI) + "[deg], u: " + String(u);
    printString += ", u_L: " + String(uL) + ", u_R: " + String(uR);
  } 
  return printString;
}

String SurfaceControl::printWaypointUpdate(void) {
  String wayPointUpdate = "";
  if (!navigateState) {
    wayPointUpdate += "SurfaceControl: Not in navigate state";
  }
  else if (!gpsAcquired) {
    wayPointUpdate += "SurfaceControl: Waiting for 6+ sats...";
  }
  else if (delayed) {
    wayPointUpdate += "SurfaceControl: Waiting for delay " + String(currentWayPoint);
  }
  else {
    wayPointUpdate += "SurfaceControl: Current WP: " + String(currentWayPoint) + "; Dist: " + String(dist) + "[m]";
  }
  return wayPointUpdate;
}

void SurfaceControl::updatePoint(float x, float y) {
  if (currentWayPoint == totalWayPoints) return; 

  float x_des = getWayPoint(0);
  float y_des = getWayPoint(1);
  dist = sqrt(pow(x-x_des,2) + pow(y-y_des,2));

  if ((dist < SUCCESS_RADIUS && currentWayPoint < totalWayPoints) || delayed) {
    String changingWPMessage = "";
    int cwpmTime = 20;

    if (delayStartTime == 0) delayStartTime = currentTime;
    if (currentTime < delayStartTime + navigateDelay) {
      delayed = 1;
      changingWPMessage = "Got to surface waypoint " + String(currentWayPoint) + ", waiting until delay is over";
    }
    else {
      delayed = 0;
      delayStartTime = 0;
      changingWPMessage = "Got to surface waypoint " + String(currentWayPoint) + ", now directing to next point";
      atPoint = 1;
      currentWayPoint++;
      
      // *** CRITICAL FIX: RESET INTEGRAL FOR NEW LEG ***
      error_integral = 0; 
    }
    
    if (currentWayPoint == totalWayPoints) {
      changingWPMessage = "Completed the surface path.";
      uR=0;
      uL=0;
      complete = 1;
      cwpmTime = 10;
    }
    printer.printMessage(changingWPMessage,cwpmTime);
  }
}

size_t SurfaceControl::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = u;
  data_slot[1] = uL;
  data_slot[2] = uR;
  data_slot[3] = yaw;
  data_slot[4] = yaw_des;
  data_slot[5] = error_integral; 
  return idx + 6*sizeof(float);
}