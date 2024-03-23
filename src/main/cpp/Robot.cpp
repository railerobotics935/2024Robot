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
 * TODO: Before comp 2
 *    TODO: Add deadzone for filtering the apriltag angle for commands sensitive to the robot angle.
 * 
 * TROUBLESHOOT: Create Apriltag recongition system 
 *    TODO: Processing speed NEEDS to be faster
 *    TODO: Increase Stability in script
 *    TODO: Increase Stability in the robot
 * 
 * LATER: Other Commands
 *    TODO: Standard Shoot
 *    TODO: Pathfind to Pose
 *    TODO: Shoot into Amp
 * 
 * DONE: Add Saftey Feautres
 *    DONE: linear Acturator
 *      DONE: position limit?
 * 
 * TODO: Create custom Command for shooting
 *    TODO: Create Auto aim/shoot command
 *      DONE: Learn how to make a command/what type
 *      TODO: Collect data on shooter to figure out what angle our shooter needs to be at for shooting
 *      TODO: Use data to auto aim
 *      TODO: Use sensor data to determine when to shoot note
 *    TODO: check location
 *      TODO: Use Pose/camera to deterime correct position
 *    VEIRIFY: Shoot While moving
 *      DONE: Math util to get robot velocity and acceleration (maybe)
 *      DONE: Look up time based on speed of shooter (we want as high of a speed as possible)
 *      DONE: use time to calculate translation of robot based on velocity and acceleration
 *      DONE: Add translation of robot in the future to current translation of the goal
 *        DONE: add method to get translation, not only distance, from goal
 *      DONE: With new distnace, re-calculate process until within an accepted error - iffy, only really works with short difference in shoot time.
 *      DONE: With final distance, grab angle/speed of shooter and set it to subsystems
 *      DONE: Rumble/automaticly shoot when good
 *      TODO: Able to set speed and angle early, but only rotate robot when almost ready to shoot to keep our options open
 *      DONE: Add M2etond to DriveSubsystem to still move using controller inputs, but stay at a commanded angle
 *      DONE: Add method to see if the pid controlle for the rotation of the robot is at the correct location
 *      DONE: Convert from tigitme to acceptiable distance, so it is based on speed
 * 
 * TODO: Improvements
 *    DONE: Work on fixing the abolute encoder problem with the swerve
 *    DONE: Implement rev through bore encoder
 *    MAYBE: Fix calibration and stability of image for apriltags
 *    DONE: Command Bindings get stuck with controller switches
 *    FIXED: Chooser to deterime if we need to burn the flash on the motorcontrollers
 *    TODO: Fix warning about construction fo motorcontrollers using constant values
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
