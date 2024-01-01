/**
 * @file relative_attitude_sensor.cpp
 * @brief Relative attitude sensor
 */

#include "relative_attitude_sensor.hpp"

#include <components/base/initialize_sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativeAttitudeSensor::RelativeAttitudeSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id,
                                               const int reference_sat_id, const RelativeInformation& rel_info, const Dynamics& dynamics)
    : Component(prescaler, clock_gen),
      Sensor(sensor_base),
      target_sat_id_(target_sat_id),
      reference_sat_id_(reference_sat_id),
      rel_info_(rel_info),
      dynamics_(dynamics) {}

RelativeAttitudeSensor::~RelativeAttitudeSensor() {}

void RelativeAttitudeSensor::MainRoutine(int count) {
  UNUSED(count);

  // Get true value
  measured_target_attitude_rb2tb_quaternion_ = rel_info_.GetRelativeAttitudeQuaternion(target_sat_id_, reference_sat_id_);
  measured_target_attitude_rb2tb_rad_ = measured_target_attitude_rb2tb_quaternion_.ConvertToEuler();
  measured_target_attitude_rb2tb_rad_ = Measure(measured_target_attitude_rb2tb_rad_);
  measured_target_attitude_rb2tb_quaternion_ = libra::Quaternion::ConvertFromEuler(measured_target_attitude_rb2tb_rad_);
}

std::string RelativeAttitudeSensor::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeAttitudeSensor_";

  const std::string frame_name = std::to_string(reference_sat_id_) + "to" + std::to_string(target_sat_id_);
  str_tmp += WriteQuaternion(head + "quaternion", frame_name);
  str_tmp += WriteVector(head + "attitude", frame_name, "rad", 3);

  return str_tmp;
}

std::string RelativeAttitudeSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteQuaternion(measured_target_attitude_rb2tb_quaternion_);
  str_tmp += WriteVector(measured_target_attitude_rb2tb_rad_);

  return str_tmp;
}

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
