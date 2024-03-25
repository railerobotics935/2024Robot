
#pragma once

#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/DriveSubsystem.h"

class VisionIntake
  : public frc2::CommandHelper<frc2::Command, VisionIntake> {
public:
  /**
   * Creates a new VisionIntake.
   *
   * @param intake The pointer to the intake subsystem
   * @param stager The pointer to the intake subsystem
   * @param drive The pointer to the drive subsystem
   * @param opController The pointer to the drive controller
   */
  explicit VisionIntake(IntakeSubsystem* intake, StagerSubsystem* stager, DriveSubsystem* drive);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:

  IntakeSubsystem* m_intake;
  StagerSubsystem* m_stager;
  DriveSubsystem* m_drive;

  int m_bestNoteId;

};
