
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ClimberSubsystem.h"

class ExtendClimber
  : public frc2::CommandHelper<frc2::Command, ExtendClimber> {
public:
  /**
   * Creates a new ExtendClimber.
   *
   * @param climber The pointer to the intake subsystem
   */
  explicit ExtendClimber(ClimberSubsystem* climber);

  void Execute() override;
  void End(bool interrupted) override;
  
private:
  ClimberSubsystem* m_climber;
};
