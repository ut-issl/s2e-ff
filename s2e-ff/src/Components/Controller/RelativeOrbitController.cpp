#include "RelativeOrbitController.hpp"

RelativeOrbitController::RelativeOrbitController(const int prescaler, ClockGenerator* clock_gen, FfComponents& components)
    : ComponentBase(prescaler, clock_gen), components_(components) {}

RelativeOrbitController::~RelativeOrbitController() {}

void RelativeOrbitController::MainRoutine(int count) { UNUSED(count); }

std::string RelativeOrbitController::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string RelativeOrbitController::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
