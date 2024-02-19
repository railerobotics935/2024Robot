
#include <math.h>
#include "utils/MathUtils.h"

double MathUtils::SignedSquare(double input) {
  if (input < 0.0)
    return -std::pow(input, 2);
  else
    return std::pow(input, 2);
}