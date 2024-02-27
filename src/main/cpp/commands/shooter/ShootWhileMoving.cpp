
#include "commands/shooter/ShootWhileMoving.h"
#include "utils/MathUtils.h"

ShootWhileMoving::ShootWhileMoving(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController, frc::XboxController* driveController) {
  // Set Local pointer
  m_shooter = shooter;
  m_drive = drive;
  m_opController = opController;
  m_driveController = driveController;

  AddRequirements(m_shooter);
  AddRequirements(m_drive);
}

// Right now nothing, but maybe somthing later
void ShootWhileMoving::Initialize() {};

void ShootWhileMoving::Execute() {

  // Get the distance and translation from the robot to the goal
  double robotToGoalDistance = MathUtils::RobotDistanceToGoal(m_drive->GetPose());
  frc::Translation2d GoalTranslation{MathUtils::TranslationToGoal(m_drive->GetPose())}; // = MathUtils::TranslationToGoal(m_drive->GetPose());

  // New empty translation to hold our calculations
  frc::Translation2d dynamicTargetTranslation{};
  frc::Translation2d robotToDynamicTargetTranslation{};
  double dynamicTargetDistance = 0.0;

  // Robot Velocity and Acceleration
  double robot_vx = m_drive->GetFieldRelativeSpeeds().vx();
  double robot_vy = m_drive->GetFieldRelativeSpeeds().vy();

  // Robot Time
  double shootingTime = ShootingCalculations::GetTimeFromDistnace(robotToGoalDistance);

  // For loop for iterations and getting more accurate mesurements
  for (int i = 0; i < 3; i++) {
    
    // Calculate the moving goal x and y
    // Right now it is not using acceleration becasue there is no limit on max velocity
    double dynamicTargetX = (double)GoalTranslation.X() - (robot_vx * shootingTime);
    double dynamicTargetY = (double)GoalTranslation.Y() - (robot_vy * shootingTime);

    // Create location of the field of moving goal
    dynamicTargetTranslation = frc::Translation2d{(units::meter_t)dynamicTargetX, (units::meter_t)dynamicTargetY};

    // Deterime the tranlsation of the robot based on where the robot will be when we shoot the note
    robotToDynamicTargetTranslation = dynamicTargetTranslation.operator-(m_drive->GetPose().Translation());

    // Get the new shot time based on the distance of the new 
    dynamicTargetDistance = MathUtils::RobotDistanceToGoal({robotToDynamicTargetTranslation, frc::Rotation2d{}});
    double newShootingTime = ShootingCalculations::GetTimeFromDistnace(dynamicTargetDistance);

    // If the difference in shoot times is close enough, end the iterations
    // TODO: Convert from time to acceptiable distance, so it is based on speed
    if (fabs(newShootingTime - shootingTime) <= 0.010) {
      i = 4; // This will end the iterations
    }
    
    // Set the shooting time to the new shooting time for the next iteration.
    shootingTime = newShootingTime;

  } // for loop

  // Set subsystems to location
  m_shooter->SetShooterAngle((units::radian_t)ShootingCalculations::GetAngleFromDistance(dynamicTargetDistance));
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)ShootingCalculations::GetSpeedFromDistance(dynamicTargetDistance));

  // Get x and y speed from controller
  const auto xSpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.05);
  const auto ySpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.05);

  m_drive->DriveFacingGoal((units::meters_per_second_t)xSpeed, (units::meters_per_second_t)ySpeed, MathUtils::AngleToGoal(robotToDynamicTargetTranslation), true);
  
  // Rumble controllers if at the setpoints
  if (m_shooter->AtSpeedSetpoint() && m_shooter->AtAngleSetpoint()) {
    m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
  }
  else {
    m_opController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
  }

  // Checking if drive is at correct angle 
  if (m_drive->AtAngleSetpoint()) {
    m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
  }
  else {
    m_opController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
  }
}

void ShootWhileMoving::End(bool interrupted) {
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)0.0);
  m_opController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
}