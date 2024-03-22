
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"

class SimpleIntake
  : public frc2::CommandHelper<frc2::Command, SimpleIntake> {
public:
  /**
   * Creates a new SimpleIntake.
   *
   * @param intake The pointer to the intake subsystem
   * @param opController The pointer to the operator controller
   */
  explicit SimpleIntake(IntakeSubsystem* intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
};
