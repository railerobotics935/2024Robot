
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class PanicOutake
  : public frc2::CommandHelper<frc2::Command, PanicOutake> {
public:
  /**
   * Creates a new PanicOutake.
   *
   * @param intake The pointer to the intake subsystem
   * @param stager The pointer to the stager subsystem
   * @param shooter The pointer to the stagersubsystem
   * @param opController The pointer to the operator controller
   */
  explicit PanicOutake(IntakeSubsystem* intake, StagerSubsystem* stager, ShooterSubsystem* shooter);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  StagerSubsystem* m_stager;
  ShooterSubsystem* m_shooter;
};
