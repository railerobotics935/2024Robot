
#include "commands/shooter/ManualCloseShoot.h"

ManualCloseShoot::ManualCloseShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void ManualCloseShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ManualCloseShoot Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)5000);
}

void ManualCloseShoot::End(bool interrupted) {
  // Reset everything to zero
  m_shooter->SetShooterAngle((units::radian_t)0.8);
  m_shooter->SetShooterMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "ManualCloseShoot Ended\r\n";
#endif
}