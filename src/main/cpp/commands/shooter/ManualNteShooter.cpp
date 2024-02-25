
#include "commands/shooter/ManualNteShooter.h"

ManualNteShooter::ManualNteShooter(ShooterSubsystem* shooter, frc::XboxController* opController) : m_shooter{shooter}, 
                                                                                                   m_opController{opController} {
  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("Set Shooter Values");

  nte_setpointSpeedRPM = nt_table->GetEntry("Setpoint Speed in RPM");
  nte_setpointAngleRadians = nt_table->GetEntry("Setpoint Angle in Radians");

  // Add requierment for subsystem
  AddRequirements(m_shooter);
}

void ManualNteShooter::Execute() {
  // Set shooter to angle and speed from shuffleboard
  m_shooter->SetShooterAngle((units::radian_t)nte_setpointAngleRadians.GetDouble(1.0));
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)nte_setpointSpeedRPM.GetDouble(0.0));

  // If at angle setpoint, rumble left
  if (m_shooter->AtAngleSetpoint())
    m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
  else 
    m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);

  // If at speed setpoint, rumble right
  if (m_shooter->AtSpeedSetpoint())
    m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
  else 
    m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
}

void ManualNteShooter::End(bool interrupted) {
  // Reset everything to zero
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterMotorPower(0.0);
  m_opController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}