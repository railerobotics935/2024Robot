
#include "Constants.h"
#include "commands/intake/StopIntake.h"

StopIntake::StopIntake(IntakeSubsystem* intake) : m_intake{intake} {
  AddRequirements(m_intake);
}

void StopIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StopIntake Initialized\r\n";
#endif

  m_intake->SetMotorPower(0.0);
}

void StopIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StopIntake Ended\r\n";
#endif
}