// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "utils/MathUtils.h"
#include "Constants.h"
#include "commands/drive/DriveFacingGoal.h"

DriveFacingGoal::DriveFacingGoal(DriveSubsystem* drive, frc::XboxController* driveController)
    : m_drive{drive}, m_driveController{driveController} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void DriveFacingGoal::Initialize() {
  // Run once when command is scheduled
  m_gyroOffset = ((double)m_drive->GetPose().Rotation().Radians()) - ((double)m_drive->GetHeading() * std::numbers::pi / 180.0);
#ifdef PRINTDEBUG
  std::cout << "DriveFacingGoal Initialized\r\n";
#endif
}

void DriveFacingGoal::Execute() {
  // Main execute loop that runs during the command
  const auto xSpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.05);
  const auto ySpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.05);
  
  m_drive->DriveFacingGoal(units::meters_per_second_t{MathUtils::SignedSquare(xSpeed)},
    units::meters_per_second_t{MathUtils::SignedSquare(ySpeed)}, //frc::Rotation2d{}.operator-((units::radian_t)m_gyroOffset),
    MathUtils::AngleToGoal(MathUtils::TranslationToGoal(m_drive->GetPose())).operator-((units::radian_t)m_gyroOffset), 
    true);

  if (m_drive->AtAngleSetpoint())
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
  else
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}

void DriveFacingGoal::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "DriveFacingGoal ended\r\n";
#endif
  m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.0);
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
}