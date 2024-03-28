
#pragma once

#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/DriveSubsystem.h"
/*
class VisionIntake
  : public frc2::CommandHelper<frc2::ParallelCommandGroup, VisionIntake> {
public:
  /**
   * Creates a new VisionIntake.
   *
   * @param intake The pointer to the intake subsystem
   * @param stager The pointer to the intake subsystem
   * @param drive The pointer to the drive subsystem
   *
  explicit VisionIntake(IntakeSubsystem* intake, StagerSubsystem* stager, DriveSubsystem* drive);

private:

  IntakeSubsystem* m_intake;
  StagerSubsystem* m_stager;
  DriveSubsystem* m_drive;

  int m_bestNoteId;
  std::vector<frc::Pose2d> m_pathPoses;
  std::vector<frc::Translation2d> m_bezierPoints;
  std::optional<frc2::CommandPtr> m_pathCommand;
};
*/