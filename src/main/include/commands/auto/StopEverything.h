
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include <subsystems/StagerSubsystem.h>
#include <subsystems/ShooterSubsystem.h>

class StopEverything
  : public frc2::CommandHelper<frc2::Command, StopEverything> {
public:
  /**
   * Creates a new StopEverything.
   * 
   * @param stager The pointer to the stager subsystem
   * @param shooter The pointer to the shooter subsystem
  */
  explicit StopEverything(StagerSubsystem* stager, ShooterSubsystem* shooter);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  StagerSubsystem* m_stager;
  ShooterSubsystem* m_shooter;
  bool m_ran = false;
};