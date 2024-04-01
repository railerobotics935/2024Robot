
#include "commands/auto/SetSmartShooterSpeeds.h"

SetSmartShooterSpeeds::SetSmartShooterSpeeds(ShooterSubsystem* shooter, DriveSubsystem* drive) {
  // Initialize local copies of pointers
  m_shooter = shooter;
  m_drive = drive;

  // Add requirements for the command
  AddRequirements(m_shooter);
}

void SetSmartShooterSpeeds::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SetSmartShooterSpeeds Initialized\r\n";
#endif
  m_robotDistance = m_drive->RobotDistanceToGoal(m_drive->GetPose());
  m_shooter->SetShooterAngle((units::radian_t)ShootingCalculations::GetAngleFromDistance(m_robotDistance));
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)ShootingCalculations::GetSpeedFromDistance(m_robotDistance));
}

bool SetSmartShooterSpeeds::IsFinished() {
  return false;
}

void SetSmartShooterSpeeds::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SetSmartShooterSpeeds Ended\r\n";
#endif
  m_ran = false;
}