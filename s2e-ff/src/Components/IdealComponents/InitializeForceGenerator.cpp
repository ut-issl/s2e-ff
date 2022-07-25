#include "InitializeForceGenerator.hpp"

#include <Interface/InitInput/IniAccess.h>

ForceGenerator InitializeForceGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("ComponentBase", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // ForceGenerator
  char section[30] = "ForceGenerator";
  double force_magnitude_standard_deviation_N = ini_file.ReadInt(section, "force_magnitude_standard_deviation_N");
  double force_direction_standard_deviation_deg = ini_file.ReadInt(section, "force_direction_standard_deviation_deg");

  ForceGenerator force_generator(prescaler, clock_gen, force_magnitude_standard_deviation_N, force_direction_standard_deviation_deg, dynamics);

  return force_generator;
}