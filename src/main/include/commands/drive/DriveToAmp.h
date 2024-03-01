
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "pathplanner/lib/path/PathPlannerPath.h"

#include "subsystems/DriveSubsystem.h"

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
  DriveSubsystem* m_drive;

};