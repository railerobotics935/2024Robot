
#include "commands/intake/SmartOuttake.h"


SmartOuttake::SmartOuttake(IntakeSubsystem* intake, StagerSubsystem* stager) {
  // Initilize local copys of pointers
  m_intake = intake;
  m_stager = stager;

  // Add reqierments for the command
  AddRequirements(m_intake);
  AddRequirements(m_stager);
}

void SmartOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SmartOuttake Initialized\r\n";
#endif
  m_intake->SetMotorPower(-1.0);
  m_stager->SetMotorPower(-0.5);
}

void SmartOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SmartOuttake Ended\r\n";
#endif
  m_intake->SetMotorPower(0.0);
  m_stager->SetMotorPower(0.0);
}