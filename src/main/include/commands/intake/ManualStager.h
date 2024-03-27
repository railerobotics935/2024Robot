
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/StagerSubsystem.h"

class ManualStager
  : public frc2::CommandHelper<frc2::Command, ManualStager> {
public:
  /**
   * Creates a new ManualStager.
   *
   * @param stager The pointer to the intake subsystem
   * @param operatorController the pointer to the operator controller
   */
  explicit ManualStager(StagerSubsystem* stager, frc::XboxController* operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  StagerSubsystem* m_stager;
  frc::XboxController* m_operatorController;
};
