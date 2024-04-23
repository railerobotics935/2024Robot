
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/StagerSubsystem.h"
#include "subsystems/DriveSubsystem.h"

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
  explicit SmartOuttake(IntakeSubsystem* intake, DriveSubsystem* drive, ShooterSubsystem* shooter, StagerSubsystem* stager, frc::XboxController* opController);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  ShooterSubsystem* m_shooter;
  StagerSubsystem* m_stager;
  DriveSubsystem* m_drive;

  frc::XboxController* m_opController;
};
