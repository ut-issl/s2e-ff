/**
 * @file initialize_relative_velocity_sensor.cpp
 * @brief Initialize function for RelativeVelocitySensor
 */

#include "initialize_relative_position_sensor.hpp"

#include <components/base/initialize_sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativePositionSensor InitializeRelativePositionSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics,
                                                        const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("ComponentBase", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // SensorBase
  Sensor<3> sensor_base = ReadSensorInformation<3>(file_name, compo_step_time_s * (double)(prescaler), "RelativePositionSensor");

  // RelativePositionSensor
  char section[30] = "RelativePositionSensor";
  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }

  std::string error_frame_string = ini_file.ReadString(section, "error_frame");
  RelativePositionSensorErrorFrame error_frame;

  if (error_frame_string == "INERTIAL") {
    error_frame = RelativePositionSensorErrorFrame::INERTIAL;
  } else if (error_frame_string == "RTN") {
    error_frame = RelativePositionSensorErrorFrame::RTN;
  } else if (error_frame_string == "BODY") {
    error_frame = RelativePositionSensorErrorFrame::BODY;
  } else {
    std::cerr << "Warnings: InitializeRelativePositionSensor: The error frame setting was failed. It is automatically set as BODY frame."
              << std::endl;
    error_frame = RelativePositionSensorErrorFrame::BODY;
  }

  RelativePositionSensor relative_position_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, error_frame, rel_info,
                                                  dynamics);

  return relative_position_sensor;
}
