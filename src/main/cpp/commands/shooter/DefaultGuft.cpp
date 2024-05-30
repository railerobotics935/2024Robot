
#include "commands/shooter/DefaultGuft.h"

DefaultGuft::DefaultGuft(GuftSubsystem* guft) : m_guft{guft} {

  // Add requierment for subsystem
  AddRequirements(m_guft);
}

void DefaultGuft::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "DefaultGuft Initialized\r\n";
#endif
  m_guft->SetGuftAngle((units::radian_t)0.43);
}

void DefaultGuft::End(bool interrupted) {
  // Reset everything to zero
  m_guft->SetGuftAngle((units::radian_t)0.43);
#ifdef PRINTDEBUG
  std::cout << "DefaultGuft Ended\r\n";
#endif
}