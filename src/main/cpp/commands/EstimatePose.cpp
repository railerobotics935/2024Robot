
#include "commands/EstimatePose.h"
#include "math.h"

EstimatePose::EstimatePose(DriveSubsystem* drive) : m_drive{drive} {/*No reqierments to add*/}

void EstimatePose::Execute() {
  // Add vision data if the robot is going slow enough
  if (m_drive->GetLinearRobotSpeed() < CameraConstants::kMaxEstimationSpeed) {
    m_drive->EstimatePoseWithApriltag();
    printf("estimating\r\n");
  }
}
