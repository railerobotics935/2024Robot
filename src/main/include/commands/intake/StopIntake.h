
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"

class StopIntake
  : public frc2::CommandHelper<frc2::Command, StopIntake> {
public:
  /**
   * Creates a new StopIntake.
   *
   * @param intake The pointer to the intake subsystem
   */
  explicit StopIntake(IntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
};
