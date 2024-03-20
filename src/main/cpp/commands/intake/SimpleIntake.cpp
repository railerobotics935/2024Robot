
#include "Constants.h"
#include "commands/intake/SimpleIntake.h"

SimpleIntake::SimpleIntake(IntakeSubsystem* intake) : m_intake{intake} {

  AddRequirements(m_intake);
}

void SimpleIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif

  m_intake->SetMotorPower(1.0);
}
void SimpleIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

  m_intake->SetMotorPower(0.0);
}