// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "utils/MathUtils.h"
#include "Constants.h"
#include "commands/SlowDrive.h"

SlowDrive::SlowDrive(DriveSubsystem* drive, frc::XboxController* driveController)
    : m_drive{drive}, m_driveController{driveController} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void SlowDrive::Initialize() {
  // Run once when command is scheduled
  printf("SlowDrive Initialized\r\n");
}

void SlowDrive::Execute() {
  // Main execute loop that runs during the command
  const auto xSpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.05) * 0.25;
  const auto ySpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.05) * 0.25;
  const auto rot = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveRightXIndex), 0.05) * 0.25;
  
  m_drive->Drive(units::meters_per_second_t{MathUtils::SignedSquare(xSpeed)},
    units::meters_per_second_t{MathUtils::SignedSquare(ySpeed)},
    units::radians_per_second_t{MathUtils::SignedSquare(rot)}, 
    true);
}

bool SlowDrive::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void SlowDrive::End(bool interupted) {
  // Runs once when the command is removed from the command scheduler
  printf("SlowDrive ended\r\n");
}