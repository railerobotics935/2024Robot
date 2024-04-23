
#include "commands/shooter/ManualFarShoot.h"

ManualFarShoot::ManualFarShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {

  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void ManualFarShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ManualFarShoot Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.3);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)1500.0);
}

void ManualFarShoot::End(bool interrupted) {
  // Reset everything to zero
#ifdef PRINTDEBUG
  std::cout << "ManualFarShoot Ended\r\n";
#endif
}