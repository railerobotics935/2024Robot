
#include "commands/shooter/LoftShoot.h"

LoftShoot::LoftShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {

  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void LoftShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "LoftShoot Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.27);
  m_shooter->SetIndividualShooterSpeed((units::revolutions_per_minute_t)2000.0, (units::revolutions_per_minute_t)2000.0);
}

void LoftShoot::End(bool interrupted) {
  // Reset everything to zero
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "LoftShoot Ended\r\n";
#endif
}