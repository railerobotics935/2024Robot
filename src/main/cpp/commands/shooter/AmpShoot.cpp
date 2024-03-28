
#include "commands/shooter/AmpShoot.h"

AmpShoot::AmpShoot(ShooterSubsystem* shooter, StagerSubsystem* stager) : m_shooter{shooter}, m_stager{stager} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
  AddRequirements(m_stager);
}

void AmpShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.7);
  m_shooter->SetIndividualShooterSpeed((units::revolutions_per_minute_t)1600, (units::revolutions_per_minute_t)300);
}

void AmpShoot::Execute() {
  if ((double)m_shooter->GetShooterAngle() > std::numbers::pi/2)
    m_stager->SetMotorPower(-1.0);
}

bool AmpShoot::IsFinished() {
  if ((double)m_shooter->GetShooterAngle() > 1.72)
    return true;
  else
    return false;
}
void AmpShoot::End(bool interrupted) {
  // Reset everything to zero
  m_shooter->SetShooterAngle((units::radian_t)0.8);
  m_shooter->SetShooterMotorPower(0.0);
  m_stager->SetMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "AmpShoot Ended\r\n";
#endif
}