
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/DriveSubsystem.h"

#include "Constants.h"

class SmartShooter  
  : public frc2::CommandHelper<frc2::Command, SmartShooter> {
public:
  /**
   * Command to shoot automaticly. Uses the robot position to 
   * 
   * @param shooter Pointer to the shooter subsytem
   * @param drive Pointer to the drive subsystem
   * @param opController Pointer to the operator controller
   * @param driveController Pointer to the driver controller
  */
  explicit SmartShooter(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController, frc::XboxController* driveController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;

private:
  // Local pointers to subsytems
  ShooterSubsystem* m_shooter;
  DriveSubsystem* m_drive;
  frc::XboxController* m_opController;
  frc::XboxController* m_driveController;

   double m_distanceToShooter;
};