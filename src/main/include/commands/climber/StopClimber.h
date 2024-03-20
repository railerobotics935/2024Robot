
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ClimberSubsystem.h"

class StopClimber
  : public frc2::CommandHelper<frc2::Command, StopClimber> {
public:
  /**
   * Creates a new StopClimber.
   *
   * @param climber The pointer to the intake subsystem
   */
  explicit StopClimber(ClimberSubsystem* climber);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ClimberSubsystem* m_climber;
};
