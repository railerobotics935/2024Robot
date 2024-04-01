
#include "commands/auto/SetCloseShooterSpeeds.h"

SetCloseShooterSpeeds::SetCloseShooterSpeeds(ShooterSubsystem* shooter) {
  // Initialize local copies of pointers
  m_shooter = shooter;

  // Add requirements for the command
  AddRequirements(m_shooter);
}

void SetCloseShooterSpeeds::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SetCloseShooterSpeeds Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)5000.0);
  m_ran = true;
}

bool SetCloseShooterSpeeds::IsFinished() {
  if (m_ran == true)
    return true;
  else
    return false;
}

void SetCloseShooterSpeeds::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SetCloseShooterSpeeds Ended\r\n";
#endif
  m_ran = false;
}