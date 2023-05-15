#include "initialize_relative_distance_sensor.hpp"

#include <components/base/initialize_sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativeDistanceSensor InitializeRelativeDistanceSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);

  // Compo
  int prescaler = ini_file.ReadInt("ComponentBase", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // Sensor
  Sensor<1> sensor_base = ReadSensorInformation<1>(file_name, compo_step_time_s * (double)(prescaler), "RelativeDistanceSensor");

  // RelativeDistanceSensor
  char section[30] = "RelativeDistanceSensor";
  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }
  RelativeDistanceSensor relative_distance_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, rel_info);

  return relative_distance_sensor;
}