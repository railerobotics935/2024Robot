

#include "commands/drive/DriveToAmp.h"

DriveToAmp::DriveToAmp(DriveSubsystem* drive) : m_drive{drive} {
  // Add requierments to the command
  AddRequirements(m_drive);
}

void DriveToAmp::Initialize() {
  // Will probably be used to create path in the moment
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