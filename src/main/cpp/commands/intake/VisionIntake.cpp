#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "commands/intake/VisionIntake.h"
#include "commands/intake/SmartIntake.h"
/*
VisionIntake::VisionIntake(IntakeSubsystem* intake, StagerSubsystem* stager, DriveSubsystem* drive) {
  // Initilize local copys of pointers
  m_intake = intake;
  m_stager = stager;
  m_drive = drive;

  // run command to get best note id and grab the pose at the same time
  m_pathPoses = {m_drive->GetPose(), frc::Pose2d{m_drive->GetFieldRelativeTranslation(m_drive->GetBestNoteId()), m_drive->GetPose().Rotation()}};

  auto path = std::make_shared<pathplanner::PathPlannerPath>(
      pathplanner::PathPlannerPath::bezierFromPoses(m_pathPoses),
      pathplanner::PathConstraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      pathplanner::GoalEndState(1.0_mps, m_drive->GetPose().Rotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );
  m_pathCommand = pathplanner::AutoBuilder::followPathWithEvents(path);

  // Add commands to run in parallel
  AddCommands(
    *m_pathCommand.value().get(), // feels really dangorous
    SmartIntake{m_intake, m_stager}
  ); 

  #ifdef PRINTDEBUG
  std::cout << "VisionIntake Initialize\r\n";
  #endif
}
*/