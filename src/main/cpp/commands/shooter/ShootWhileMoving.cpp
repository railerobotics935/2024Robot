
#include "commands/shooter/ShootWhileMoving.h"
#include "utils/MathUtils.h"

ShootWhileMoving::ShootWhileMoving(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController) {
  // Set Local pointer
  m_shooter = shooter;
  m_drive = drive;
  m_opController = opController;

  AddRequirements(m_shooter);
}

// Right now nothing, but maybe somthing later
void ShootWhileMoving::Initialize() {};

void ShootWhileMoving::Execute() {

  // Get the distance and translation from the robot to the goal
  double robotToGoalDistance = MathUtils::RobotDistanceToGoal(m_drive->GetPose());
  frc::Translation2d GoalTranslation{MathUtils::TranslationToGoal(m_drive->GetPose())}; // = MathUtils::TranslationToGoal(m_drive->GetPose());

  // New empty translation to hold our calculations
  frc::Translation2d movingGoalTranslation{};
  double movingGoalDistance = 0.0;

  // Robot Velocity and Acceleration
  double robot_vx = 0.0;
  double robot_vy = 0.0;
  double robot_ax = 0.0;
  double robot_ay = 0.0;

  // Robot Time
  double shootingTime = DataCurve::GetTimeFromDistnace(robotToGoalDistance);

  // For loop for iterations and getting more accurate mesurements
  for (int i = 0; i < 5; i++)
  {
    // Calculate the moving goal x and y
    // Right now it is not using acceleration becasue there is no limit on max velocity
    double movingGoalX = (double)GoalTranslation.X() - (robot_vx * shootingTime);
    double movingGoalY = (double)GoalTranslation.Y() - (robot_vy * shootingTime);

    // Create location of the field of moving goal
    movingGoalTranslation = frc::Translation2d{(units::meter_t)movingGoalX, (units::meter_t)movingGoalY};

    // Deterime the tranlsation of the robot based on where the robot will be when we shoot the note
    frc::Translation2d robotToMovingGoalTranslation = movingGoalTranslation.operator-(m_drive->GetPose().Translation());

    // Get the new shot time based on the distance of the new 
    movingGoalDistance = MathUtils::RobotDistanceToGoal({robotToMovingGoalTranslation, frc::Rotation2d{}});
    double newShootingTime = DataCurve::GetTimeFromDistnace(movingGoalDistance);

    // If the difference in shoot times is close enough, end the iterations
    if (fabs(newShootingTime - shootingTime) <= 0.010) {
      i = 4; // This will end the iterations
    }
    
    // Set the shooting time to the new shooting time for the next iteration.
    shootingTime = newShootingTime;

  } // for loop

  // Set subsystems to location
  m_shooter->SetShooterAngle((units::radian_t)DataCurve::GetAngleFromDistance(movingGoalDistance));
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)DataCurve::GetSpeedFromDistance(movingGoalDistance));
  // Set drivetrain to angle of goal
  
  // Rumble controllers if at the setpoints
  
}

void ShootWhileMoving::End(bool interrupted) {
  m_shooter->SetShooterAngle((units::radian_t)1.0);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)0.0);
  m_opController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}