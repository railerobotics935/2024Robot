
#include "commands/shooter/ManualCloseShoot.h"

ManualCloseShoot::ManualCloseShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void ManualCloseShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ManualCloseShoot Initialized\r\n";
#endif
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)1500);
  m_shooter->SetShooterAngle((units::radian_t)0.9);

}

void ManualCloseShoot::End(bool interrupted) {
  // Reset everything to zero
#ifdef PRINTDEBUG
  std::cout << "ManualCloseShoot Ended\r\n";
#endif
}