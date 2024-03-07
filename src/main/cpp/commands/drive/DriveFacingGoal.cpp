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
  printf("DriveFacingGoal Initialized\r\n");
}

void DriveFacingGoal::Execute() {
  // Main execute loop that runs during the command
  const auto xSpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.05);
  const auto ySpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.05);
  
  m_drive->DriveFacingGoal(units::meters_per_second_t{MathUtils::SignedSquare(xSpeed)},
    units::meters_per_second_t{MathUtils::SignedSquare(ySpeed)},
    MathUtils::AngleToGoal(m_drive->GetPose().Translation()), 
    true);

  if (m_drive->AtAngleSetpoint())
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
  else
    m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
}

bool DriveFacingGoal::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void DriveFacingGoal::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
  printf("DriveFacingGoal ended\r\n");
  m_driveController->SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.0);
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
}