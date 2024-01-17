/**
 * @file initialize_relative_distance_sensor.cpp
 * @brief Relative distance sensor
 */

#include "initialize_relative_distance_sensor.hpp"

#include <components/base/sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativeDistanceSensor InitializeRelativeDistanceSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);
  char section[30] = "RELATIVE_DISTANCE_SENSOR";

  // RelativeDistanceSensor
  int prescaler = ini_file.ReadInt(section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }

  // Sensor
  Sensor<1> sensor_base = ReadSensorInformation<1>(file_name, compo_step_time_s * (double)(prescaler), section, "m");

  RelativeDistanceSensor relative_distance_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, rel_info);

  return relative_distance_sensor;
}
