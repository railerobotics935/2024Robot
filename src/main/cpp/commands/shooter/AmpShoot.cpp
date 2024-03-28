
#include "commands/shooter/AmpShoot.h"

AmpShoot::AmpShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {

  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void AmpShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)0.75);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)8500);
}

void AmpShoot::End(bool interrupted) {
  // Reset everything to zero
  m_shooter->SetShooterAngle((units::radian_t)0.0);
  m_shooter->SetShooterMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Ended\r\n";
#endif
}