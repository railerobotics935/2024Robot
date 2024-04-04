
#include "Constants.h"
#include "commands/climber/RetractClimber.h"

RetractClimber::RetractClimber(ClimberSubsystem* climber) : m_climber{climber} {
  AddRequirements(m_climber);
}

void RetractClimber::Execute() {
#ifdef PRINTDEBUG
  std::cout << "RetractClimber Initialized\r\n";
#endif

  m_climber->SetClimberPower(1.0);
}

void RetractClimber::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "RetractClimber Ended\r\n";
#endif  

  m_climber->SetClimberPower(0.0);
}