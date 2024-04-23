
#include "commands/intake/SmartOuttake.h"


SmartOuttake::SmartOuttake(IntakeSubsystem* intake, DriveSubsystem* drive, ShooterSubsystem* shooter, StagerSubsystem* stager, frc::XboxController* opController) {
  // Initilize local copys of pointers
  m_intake = intake;
  m_stager = stager;
  m_shooter = shooter;
  m_drive = drive;
  m_opController = opController;

  // Add reqierments for the command
  AddRequirements(m_intake);
  AddRequirements(m_stager);
  AddRequirements(m_shooter);

}

void SmartOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SmartOuttake Initialized\r\n";
#endif
  m_intake->SetMotorPower(-1.0);
  m_stager->SetMotorPower(-0.5);
  m_shooter->SetShooterSpeed((units::revolutions_per_minute_t)-1000.0);
  m_shooter->SetPitchMotorPower(m_opController->GetRawAxis(ControllerConstants::kOperatorLeftYIndex));

}

void SmartOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SmartOuttake Ended\r\n";
#endif
  m_intake->SetMotorPower(0.0);
  m_stager->SetMotorPower(0.0);
  m_shooter->SetShooterMotorPower(0.0);
  m_shooter->SetPitchMotorPower(0.0);
}