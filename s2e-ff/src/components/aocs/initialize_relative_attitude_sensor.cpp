/**
 * @file initialize_relative_attitude_sensor.cpp
 * @brief Initialize function for RelativeAttitudeSensor
 */

#include "initialize_relative_attitude_sensor.hpp"

#include <components/base/initialize_sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativeAttitudeSensor InitializeRelativeAttitudeSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics,
                                                        const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);
  char section[30] = "RELATIVE_ATTITUDE_SENSOR";

  // CompoBase
  int prescaler = ini_file.ReadInt(section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // RelativeAttitudeSensor
  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }

  // SensorBase
  Sensor<3> sensor_base = ReadSensorInformation<3>(file_name, compo_step_time_s * (double)(prescaler), section, "rad");

  RelativeAttitudeSensor relative_attitude_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, rel_info, dynamics);

  return relative_attitude_sensor;
}
