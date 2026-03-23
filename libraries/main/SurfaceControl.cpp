#include "SurfaceControl.h"
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

SurfaceControl::SurfaceControl(void) 
: DataSource("u,uL,uR,yaw,yaw_des","float,float,float,float,float"){}


void SurfaceControl::init(const int totalWayPoints_in, double * wayPoints_in, int navigateDelay_in) {
  totalWayPoints = totalWayPoints_in;
  // create wayPoints array on the Heap so that it isn't erased once the main Arduino loop starts
  wayPoints = new double[2*totalWayPoints]; // Create a 1-d array to hold the waypoints in the format x0,y0,x1,y1,...
  for (int i=0; i<totalWayPoints; i++) { 
    wayPoints[i] = wayPoints_in[i];
  }
  navigateDelay = navigateDelay_in;
  if (totalWayPoints == 0) atPoint = 1; // not doing surface control
  else atPoint = 0; // doing surface control
}

int SurfaceControl::getWayPoint(int dim) {
  return wayPoints[currentWayPoint*stateDims+dim];
}

void SurfaceControl::navigate(xy_state_t * state, gps_state_t * gps_state_p, int currentTime_in) {
  currentTime = currentTime_in;

  if (gps_state_p->num_sat >= N_SATS_THRESHOLD) {
    gpsAcquired = 1;

    updatePoint(state->x, state->y);
    if (currentWayPoint == totalWayPoints) return; // stops motors at final surface point
    
    if (atPoint || delayed) {
      uL = 0; 
      uR = 0;
      return; // stops motors at surface waypoint
    }

    // set up variables
    int x_des = getWayPoint(0);
    int y_des = getWayPoint(1);

    // Set the values of yaw_des, yaw, yaw_error, control effort (u), uL, and uR appropriately for P control
    // You can use trig functions (atan2 might be useful)
    // You can access the x and y coordinates calculated in XYStateEstimator.cpp using state->x and state->y respectively
    // You can access the yaw calculated in XYStateEstimator.cpp using state->yaw

    
  // 1. Store current yaw in the class variable for logging/printing
    yaw = state->yaw;

    // 2. Calculate desired yaw using atan2 (y, x)
    yaw_des = atan2(y_des - state->y, x_des - state->x);

    // 3. Calculate the error and use the included helper function to bound it between -PI and PI
    float yaw_error = angleDiff(yaw_des - yaw);

    // 4. Proportional Control math (u = Kp * error)
    u = Kp * yaw_error;

    // 5. Set motor values (using avgPower defined in SurfaceControl.h)
    // A positive yaw_error means we need to turn Left (CCW). 
    // To turn left, the Right motor needs more power.
    uR = avgPower + u;
    uL = avgPower - u;

    // 6. Apply motor balance corrections (Kr and Kl)
    uR = uR * Kr;
    uL = uL * Kl;

    // 7. Bound the outputs between 0 and 127 to protect the H-bridges
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
    printString += "SurfaceControl: Waiting to acquire more satellites...";
  }
  else {
    printString += "SurfaceControl: ";
    printString += "Yaw_Des: ";
    printString += String(yaw_des*180.0/PI);
    printString += "[deg], ";
    printString += "Yaw: ";
    printString += String(yaw*180.0/PI);
    printString += "[deg], ";
    printString += "u: ";
    printString += String(u);
    printString += ", u_L: ";
    printString += String(uL);
    printString += ", u_R: ";
    printString += String(uR);
  } 
  return printString;
}

String SurfaceControl::printWaypointUpdate(void) {
  String wayPointUpdate = "";
  if (!navigateState) {
    wayPointUpdate += "SurfaceControl: Not in navigate state";
  }
  else if (!gpsAcquired) {
    wayPointUpdate += "SurfaceControl: Waiting to acquire more satellites...";
  }
  else if (delayed) {
    wayPointUpdate += "SurfaceControl: Waiting for delay";
    wayPointUpdate += String(currentWayPoint);
  }
  else {
    wayPointUpdate += "SurfaceControl: ";
    wayPointUpdate += "Current Waypoint: ";
    wayPointUpdate += String(currentWayPoint);
    wayPointUpdate += "; Distance from Waypoint: ";
    wayPointUpdate += String(dist);
    wayPointUpdate += "[m]";
  }
  return wayPointUpdate;
}

void SurfaceControl::updatePoint(float x, float y) {
  if (currentWayPoint == totalWayPoints) return; // don't check if finished

  float x_des = getWayPoint(0);
  float y_des = getWayPoint(1);
  dist = sqrt(pow(x-x_des,2) + pow(y-y_des,2));

  if ((dist < SUCCESS_RADIUS && currentWayPoint < totalWayPoints) || delayed) {
    String changingWPMessage = "";
    int cwpmTime = 20;

    // navigateDelay
    if (delayStartTime == 0) delayStartTime = currentTime;
    if (currentTime < delayStartTime + navigateDelay) {
      delayed = 1;
      changingWPMessage = "Got to surface waypoint " + String(currentWayPoint)
        + ", waiting until delay is over";
    }
    else {
      delayed = 0;
      delayStartTime = 0;
      changingWPMessage = "Got to surface waypoint " + String(currentWayPoint)
        + ", now directing to next point";
      atPoint = 1;
      currentWayPoint++;
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
  return idx + 5*sizeof(float);
}