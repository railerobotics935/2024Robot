
#pragma once

#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>

#include <pathplanner/lib/path/PathPlannerPath.h>

#include "subsystems/DriveSubsystem.h"

/**
 * This command uses the path planner libary to create and run paths on the fly. Specificly
 * creates bezier curves to follow. 
 * 
 * Currly we might have to create our own bezier curves from scratch, as it is better custimaize 
 * and it seems to only be an array of translations, or in our case 2d points on the field.
 * 
 * link to c++ example documentation
 * https://github.com/mjansen4857/pathplanner/wiki/Cpp-Example:-Create-a-Path-On%E2%80%90the%E2%80%90fly
*/

class DriveToAmp 
  : public frc2::CommandHelper<frc2::Command, DriveToAmp> {
public:

  /**
   * Creates a DriveToAmp Command
   * 
   * Uses the robot position to create a bézier curve with path planner and 
   * move to score in the amp
   * 
   * @param drive Pointer to the drive subsystem
  */
  explicit DriveToAmp(DriveSubsystem* drive);

  void Initialize() override; // Initializes
  void Execute() override; // Main loop that runs
  bool IsFinished() override; // Can sample states to determine if command needs to end
  void End(bool interrupted) override; // Runs once after command is finnished

private:
  // Local copy of subsystem
  DriveSubsystem* m_drive;

  // Field poess and array to hold bezier points
  std::vector<frc::Pose2d> m_fieldPoses;
  std::vector<frc::Translation2d> m_bezierPoints;
  
};