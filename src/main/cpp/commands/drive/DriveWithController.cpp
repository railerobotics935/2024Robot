// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Constants.h"
#include "commands/drive/DriveWithController.h"

DriveWithController::DriveWithController(DriveSubsystem* drive, frc::XboxController* driveController)
    : m_drive{drive}, m_driveController{driveController} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void DriveWithController::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "DriveWithController Initialized\r\n";
#endif
}

void DriveWithController::Execute() {
  // Main execute loop that runs during the command
  const auto xSpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftYIndex), 0.05);
  const auto ySpeed = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveLeftXIndex), 0.05);
  const auto rot = -frc::ApplyDeadband(m_driveController->GetRawAxis(ControllerConstants::kDriveRightXIndex), 0.05);
  
  m_drive->Drive(units::meters_per_second_t{m_drive->SignedSquare(xSpeed)},
    units::meters_per_second_t{m_drive->SignedSquare(ySpeed)},
    units::radians_per_second_t{m_drive->SignedSquare(rot)}, 
    true);
}

bool DriveWithController::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void DriveWithController::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "DriveWithController Ended\r\n";
#endif
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
}