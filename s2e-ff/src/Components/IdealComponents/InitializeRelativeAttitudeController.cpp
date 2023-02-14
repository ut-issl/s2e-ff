#include "InitializeRelativeAttitudeController.hpp"

#include <Interface/InitInput/IniAccess.h>

RelativeAttitudeController InitializeRelativeAttitudeController(ClockGenerator* clock_gen, const std::string file_name,
                                                                const RelativeInformation& rel_info,
                                                                const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics,
                                                                const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("ComponentBase", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // RelativeAttitudeController
  char section[30] = "RelativeAttitudeController";

  std::string main_mode_name = ini_file.ReadString(section, "main_mode");
  RelativeAttitudeControlMode main_mode = ConvertStringToRelativeAttitudeControlMode(main_mode_name);
  std::string sub_mode_name = ini_file.ReadString(section, "sub_mode");
  RelativeAttitudeControlMode sub_mode = ConvertStringToRelativeAttitudeControlMode(sub_mode_name);

  libra::Vector<3> main_target_direction_b;
  ini_file.ReadVector(section, "main_target_direction_b", main_target_direction_b);
  libra::Vector<3> sub_target_direction_b;
  ini_file.ReadVector(section, "sub_target_direction_b", sub_target_direction_b);

  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }

  RelativeAttitudeController relative_attitude_controller(prescaler, clock_gen, main_mode, sub_mode, main_target_direction_b, sub_target_direction_b,
                                                          target_sat_id, reference_sat_id, rel_info, local_celes_info, dynamics);

  return relative_attitude_controller;
}
