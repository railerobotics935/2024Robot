
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include <subsystems/StagerSubsystem.h>

class StageForShooting
  : public frc2::CommandHelper<frc2::Command, StageForShooting> {
public:
  /**
   * Creates a new StageForShooting.
   * 
   * @param stager The pointer to the stager subsystem
  */
  explicit StageForShooting(StagerSubsystem* stager);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  StagerSubsystem* m_stager;
  bool m_ran = false;
};