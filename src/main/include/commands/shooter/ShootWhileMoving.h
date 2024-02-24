
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "subsystems/ShooterSubsystem.h"
#include "subsystems/DriveSubsystem.h"

#include "Constants.h"


class ShootWhileMoving  
  : public frc2::CommandHelper<frc2::Command, ShootWhileMoving> {
public:
  /**
   * Command to shoot while moving. Uses information from the drivesubsystem to calculate a virtual goal
   * and sets the shooter to that state
   * 
   * @param shooter Pointer to the shooter subsytem
   * @param drive Pointer to the drive subsystem
   * @param opController Pointer to the operator controller
  */
  explicit ShootWhileMoving(ShooterSubsystem* shooter, DriveSubsystem* drive, frc::XboxController* opController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;

private:
  // Local pointers to subsytems
  ShooterSubsystem* m_shooter;
  DriveSubsystem* m_drive;
  frc::XboxController* m_opController;

};