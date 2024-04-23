
#include "commands/shooter/DefaultShooter.h"

DefaultShooter::DefaultShooter(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController) : m_shooter{shooter}, m_drive{drive}, m_opController{opController} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void DefaultShooter::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "DefaultShooter Initialized\r\n";
#endif
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterMotorPower(0.0);
  printf("%f", m_opController->GetRawAxis(ControllerConstants::kOperatorLeftYIndex));
}

void DefaultShooter::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "DefaultShooter Ended\r\n";
#endif
}