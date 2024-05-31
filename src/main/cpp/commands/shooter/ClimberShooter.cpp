
#include "commands/shooter/ClimberShooter.h"

ClimberShooter::ClimberShooter(ShooterSubsystem* shooter) : m_shooter{shooter} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void ClimberShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ClimberShooter Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)0.8);
  m_shooter->SetShooterMotorPower(0.0);
}

void ClimberShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ClimberShooter Ended\r\n";
#endif
}