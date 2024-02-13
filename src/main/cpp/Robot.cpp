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
 * Last clear of done tasks 1/20
 * 
 * TODO: Create Pathplanner Auto
 *    VERIFY: Fix Gear ratios
 *    TODO: Tune PID controller for accurate positioning
 * 
 * VERIFY: Create Apriltag recongition system
 *    TODO: Finalize camera location/configuatoin
 *    DONE: Create reference list for apriltag positions on fiels / Pull information about apriltag position
 *    DONE: Transform apriltag Pose2d to get field relative robot position
 *    DONE: Correct the axis used for the apriltag to get correct pose3d
 *    DONE: Itterate for multiple cameras
 *    DONE: Fix camera
 *    TODO: Create a poseEstimator for integration of robot
 *      VERIFY: Add timestamp for pose estimator
 *
 * TODO: Add Robot Code, an overview
 *    DONE: Convert to trapazoid robot (Honeybee)
 *    DONE: Split robot into subsystems
 *    TODO: Create commands for each action needed
 *    TODO: Map keybindings for actions
 * 
 * TODO: Autos
 *    TODO: Regester Command for auto
 *    TODO: Create Autos using commands
 * 
 * DONE: Shooter Subsystem
 *    DONE: Create Basic functions
 *    DONE: Define Sensors
 *    DONE: Implement PID Controller
 * 
 * TODO: Elevator Subsystem
 *    TODO: Create Basic functions
 *    DONE: Define Sensors
 *    TODO: Implement PID Controller
 *    TODO: Check inversion for follower moter
 * 
 * TODO: Create custom Command for shooting
 *    TODO: Create Auto aim/shoot command
 *      TODO: Learn how to make a command/what type
 *      TODO: Collect data on shooter to figure out what angle our shooter needs to be at for shooting
 *      TODO: Use data to auto aim
 *      TODO: Use sensor data to determine when to shoot note
 *    TODO: check location
 *      TODO: Use Pose/camera to deterime correct position
 * 
 * TODO: Improvements
 *    DONE: Work on fixing the abolute encoder problem with the swerve
 *    DONE: Implement rev through bore encoder
 *    MAYBE: Fix calibration and stability of image for apriltags
 *    VERIFY: Command Bindings get stuck with controller switches
 *    FIXED: Chooser to deterime if we need to burn the flash on the motorcontrollers
 *    TODO: Fix warning about construction fo motorcontrollers using constant values
 *    TODO: Define the "interruptible" state for commands
 * 
 * TODO: Clean up code
 *    DONE: Fix indentation size
 *    TODO: Thoroughly comment code
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
void Robot::DisabledInit() {
    // This makes sure that the autonomous stops running when
  // Disabled. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

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
