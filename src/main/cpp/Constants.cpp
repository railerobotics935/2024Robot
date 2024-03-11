
#include "Constants.h"

namespace CameraConstants {

  double GetStandardDeviationFromDistance(double distance) {
    if(distance < 1.0)
      return kMinStandardDeviation;
    else if (distance > 10.0)
      return kMaxStandardDeviation;
    else
      return ((distance - 1.0) / 9.0) * (kMaxStandardDeviation - kMinStandardDeviation) + kMinStandardDeviation;
  }
}

namespace ShootingCalculations {
  
  // Put equations for each here
  double GetAngleFromDistance(double distance) {
    return (1.82 - (0.723 * distance) + (0.115 * pow(distance, 2.0))); // in radians
  }
  
  double GetSpeedFromDistance(double distance) {
    return 8500.0; // in RPM
  }  
  
  double GetTimeFromDistnace(double distance) {
    return 0.0; // in Seconds
  }
}