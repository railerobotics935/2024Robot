
#include <frc/DriverStation.h>

#include "commands/drive/DriveToAmp.h"
#include "Constants.h"

DriveToAmp::DriveToAmp(DriveSubsystem* drive) : m_drive{drive} {
  // Add requierments to the command
  AddRequirements(m_drive);
}

void DriveToAmp::Initialize() {
  // Create poses based on robot position and where the goal is
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    m_fieldPoses = {m_drive->GetPose(), GameConstants::kRobotPoseForBlueAmp};
  else
    m_fieldPoses = {m_drive->GetPose(), GameConstants::kRobotPoseForRedAmp};

  // create bezier points out of them
  m_bezierPoints = pathplanner::PathPlannerPath::bezierFromPoses(m_fieldPoses);
}

void DriveToAmp::Execute() {
  // Will probably be used to update robot position
}

bool DriveToAmp::IsFinished() {
  // Will probably be used

  return false;
}

void DriveToAmp::End(bool interrupted) {
  // used to reset things to zero

}