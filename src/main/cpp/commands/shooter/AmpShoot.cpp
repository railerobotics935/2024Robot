
#include "commands/shooter/AmpShoot.h"

AmpShoot::AmpShoot(ShooterSubsystem* shooter, GuftSubsystem* guft) : m_shooter{shooter}, m_guft{guft} {

  // Add requierment for subsystem
  AddRequirements(m_shooter);
  AddRequirements(m_guft);
}

void AmpShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Initialized\r\n";
#endif
  m_guft->SetGuftAngle((units::radian_t)2.93);
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