
#include "Constants.h"
#include "commands/intake/SimpleIntake.h"

SimpleIntake::SimpleIntake(IntakeSubsystem* intake) : m_intake{intake} {

  AddRequirements(m_intake);
}

void SimpleIntake::Initialize() {
  m_intake->SetMotorPower(1.0);
}
void SimpleIntake::End(bool interrupted) {
  m_intake->SetMotorPower(0.0);
}