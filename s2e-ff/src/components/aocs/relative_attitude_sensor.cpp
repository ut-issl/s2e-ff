/**
 * @file relative_attitude_sensor.cpp
 * @brief Relative attitude sensor
 */

#include "relative_attitude_sensor.hpp"

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
  measured_target_attitude_b_quaternion_ = rel_info_.GetRelativeAttitudeQuaternion(target_sat_id_, reference_sat_id_);
  measured_target_attitude_b_rad_ = measured_target_attitude_b_quaternion_.ConvertToEuler();
  measured_target_attitude_b_rad_ = Measure(measured_target_attitude_b_rad_);
  measured_target_attitude_b_quaternion_ = libra::Quaternion::ConvertFromEuler(measured_target_attitude_b_rad_);
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

  str_tmp += WriteQuaternion(measured_target_attitude_b_quaternion_);
  str_tmp += WriteVector(measured_target_attitude_b_rad_);

  return str_tmp;
}
