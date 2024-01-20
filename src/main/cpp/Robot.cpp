// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include "Robot.h"


/**
 * Bassed on an offseason commmand robot project for the 2023 season,
 * this code is the for our Robot in the 2024 frc season
 *
 * List of items need to be compleated, list may change or be amended as 
 * we decide on adding new features
 * 
 * TODO: Create Pathplanner Auto
 *    TODO: Fix Gear ratios
 *    TODO: Tune PID controller for accurate positioning
 *    TODO: Verfiy
 * 
 * TODO: Create Apriltag recongition system
 *    TODO: decide on camera location/configuatoin
 *    TODO: Fix calibration and stability of image
 *    TODO: Update the field relative robot postion bassed on the apriltag 
 *      TODO: Create reference list for apriltag positions on fiels / Pull information about apriltag position
 *      TODO: Correct the axis used for the apriltag to get correct pose3d
 *      TODO: Verify 
 *    TODO: Dynamilcy solve for mulitple cameras
 *      TODO: Determine best algorith for using multiple apriltags
 *      TODO: Implement for multiple tags
 *      TODO: Implement for multiple cameras
 *      TODO: Verify 
 *    TODO: Verfiy
 *
 * TODO: Add Robot Code
 *    TODO: Split robot into subsystems
 *    TODO: Create commands for each action needed
 *    TODO: Map keybindings for each command
 *    TODO: Regester Command for auto
 *    TODO: Create Autos using command
 *    TODO: Verfiy
 * 
 * TODO: Improvements
 *    TODO: Work on fixing the abolute encoder problem with the swerve
 *    TODO: Implement
 * 
 * TODO: Clean up code
 *    TODO: Fix indentation size
 * 
 *  Team 935
*/

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
