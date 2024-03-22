
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include <subsystems/StagerSubsystem.h>

class StopStager
  : public frc2::CommandHelper<frc2::Command, StopStager> {
public:
  /**
   * Creates a new StopStager.
   * 
   * @param stager The pointer to the stager subsystem
  */
  explicit StopStager(StagerSubsystem* stager);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  StagerSubsystem* m_stager;
  bool m_ran = false;
};