
#include "Constants.h"
#include "commands/intake/PanicOutake.h"

PanicOutake::PanicOutake(IntakeSubsystem* intake, StagerSubsystem* stager, ShooterSubsystem* shooter) {
  m_intake = intake;
  m_stager = stager;  
  m_shooter = m_shooter;

  AddRequirements(m_intake);
}

void PanicOutake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "PanicOutake Initialized\r\n";
#endif

  m_intake->SetMotorPower(1.0);
}
void PanicOutake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "PanicOutake Ended\r\n";
#endif

  m_intake->SetMotorPower(0.0);
}