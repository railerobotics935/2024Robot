
#include "commands/shooter/SmartShooting.h"
#include "commands/drive/DriveFacingGoal.h"
#include "utils/MathUtils.h"

SmartShooter::SmartShooter(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController, frc::XboxController* driveController) {
  // Set Local pointer
  m_shooter = shooter;
  m_drive = drive;
  m_opController = opController;
  m_driveController = driveController;

  AddRequirements(m_shooter);
  AddRequirements(m_drive);
}

// Right now nothing, but maybe somthing later
void SmartShooter::Initialize() {
  // DriveFacingGoal{m_drive, m_driveController}.ToPtr();
};

void SmartShooter::Execute() {
  
  // Grab speeds from the controllers
  const auto xSpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.05);
  const auto ySpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.05);
  
  // Drive while facing the goal
  m_drive->DriveFacingGoal(units::meters_per_second_t{MathUtils::SignedSquare(xSpeed)},
    units::meters_per_second_t{MathUtils::SignedSquare(ySpeed)},
    MathUtils::AngleToGoal(MathUtils::TranslationToGoal(m_drive->GetPose())), 
    true);

  // Set Shooter to correct angle based on the distance fo the robot
  m_distanceToShooter = MathUtils::RobotDistanceToGoal(m_drive->GetPose());
  m_shooter->SetShooterAngle((units::radian_t)ShootingCalculations::GetAngleFromDistance(m_distanceToShooter));
  m_shooter->SetShooterSpeed((units::radians_per_second_t)ShootingCalculations::GetSpeedFromDistance(m_distanceToShooter));

  // If all setpoints are good, as we are slow enough, rumble both controllers at max
  if (m_drive->AtAngleSetpoint() && m_shooter->AtAngleSetpoint() && m_shooter->AtSpeedSetpoint() && m_drive->GetLinearRobotSpeed() < 0.5) {
    // Rumble both controllers max
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
    m_opController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
  }
  else { // Do step processing
    // Rumble left when drive is at setpoint
    if (m_drive->AtAngleSetpoint()){
      m_driveController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.25);
      m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.25);
    }
    else {
      m_driveController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
      m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    }

    // Rumble right when shooter is at setpoints
    if (m_shooter->AtSpeedSetpoint() && m_shooter->AtAngleSetpoint()) {
      m_driveController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.25);
      m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.25);
    }
    else {
      m_driveController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
      m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
    }
  }
}

void SmartShooter::End(bool interrupted) {
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)0.0);
  m_opController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
}