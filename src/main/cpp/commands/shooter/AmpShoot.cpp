
#include "commands/shooter/AmpShoot.h"

AmpShoot::AmpShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {

  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void AmpShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.27);
  m_shooter->SetIndividualShooterSpeed((units::revolutions_per_minute_t)300.0, (units::revolutions_per_minute_t)2000.0);
}

void AmpShoot::End(bool interrupted) {
  // Reset everything to zero
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Ended\r\n";
#endif
}