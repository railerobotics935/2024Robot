
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/StagerSubsystem.h"

class SmartOuttake
  : public frc2::CommandHelper<frc2::Command, SmartOuttake> {
public:
  /**
   * Creates a new SmartOuttake.
   *
   * @param intake The pointer to the intake subsystem
   * @param stager The pointer to the intake subsystem
   * @param opController The pointer to the drive controller
   */
  explicit SmartOuttake(IntakeSubsystem* intake, StagerSubsystem* stager);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  StagerSubsystem* m_stager;
};
