
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ClimberSubsystem.h"

class RetractClimber
  : public frc2::CommandHelper<frc2::Command, RetractClimber> {
public:
  /**
   * Creates a new RetractClimber.
   *
   * @param climber The pointer to the intake subsystem
   */
  explicit RetractClimber(ClimberSubsystem* climber);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  ClimberSubsystem* m_climber;
};
