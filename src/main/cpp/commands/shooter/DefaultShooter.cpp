
#include "commands/shooter/DefaultShooter.h"

DefaultShooter::DefaultShooter(ShooterSubsystem* shooter) : m_shooter{shooter} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void DefaultShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "DefaultShooter Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)0.8);
  m_shooter->SetShooterMotorPower(0.0);
}

void DefaultShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "DefaultShooter Ended\r\n";
#endif
}