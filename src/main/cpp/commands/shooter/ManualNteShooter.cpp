
#include "commands/shooter/ManualNteShooter.h"

ManualNteShooter::ManualNteShooter(ShooterSubsystem* shooter, frc::XboxController* opController) : m_shooter{shooter}, 
                                                                                                   m_opController{opController} {
  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void ManualNteShooter::Initialize() {
  // Initializes ManualNteShooter
#ifdef PRINTDEBUG
  std::cout << "ManualNteShooter Initialized\r\n";
#endif
}

void ManualNteShooter::Execute() {
  // Set shooter to angle and speed from shuffleboard
  m_shooter->ManualNteShoot();

  // If at angle setpoint, rumble left
  if (m_shooter->AtAngleSetpoint())
    m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.5);
  else 
    m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);

  // If at speed setpoint, rumble right
  if (m_shooter->AtSpeedSetpoint())
    m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.5);
  else 
    m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0); 
}

void ManualNteShooter::End(bool interrupted) {
  // Reset everything to zero
#ifdef PRINTDEBUG
  std::cout << "ManualNteShooter Ended\r\n";
#endif

  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterMotorPower(0.0);
  m_opController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}