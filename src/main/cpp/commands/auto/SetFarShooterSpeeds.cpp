
#include "commands/auto/SetFarShooterSpeeds.h"

SetFarShooterSpeeds::SetFarShooterSpeeds(ShooterSubsystem* shooter) {
  // Initialize local copies of pointers
  m_shooter = shooter;
  // Add requirements for the command
  AddRequirements(m_shooter);
}

void SetFarShooterSpeeds::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SetFarShooterSpeeds Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)0.7);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)8500.0);
  m_ran = true;
}

bool SetFarShooterSpeeds::IsFinished() {
  if (m_ran == true)
    return true;
  else
    return false;
}

void SetFarShooterSpeeds::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SetFarShooterSpeeds Ended\r\n";
#endif
  m_ran = false;
}