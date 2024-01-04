/**
 * @file relative_attitude_sensor.cpp
 * @brief Relative attitude sensor
 */

#include "relative_attitude_sensor.hpp"

#include <components/base/initialize_sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativeAttitudeSensor::RelativeAttitudeSensor(const int prescaler, ClockGenerator* clock_gen, const int target_sat_id, const int reference_sat_id,
                                               const RelativeInformation& rel_info, const double standard_deviation_rad)
    : Component(prescaler, clock_gen),
      target_sat_id_(target_sat_id),
      reference_sat_id_(reference_sat_id),
      rel_info_(rel_info),
      angle_noise_(0.0, standard_deviation_rad) {
  direction_noise_.SetParameters(0.0, 1.0);
}

RelativeAttitudeSensor::~RelativeAttitudeSensor() {}

void RelativeAttitudeSensor::MainRoutine(int count) {
  UNUSED(count);
  // Error calculation
  libra::Vector<3> random_direction;
  random_direction[0] = direction_noise_;
  random_direction[1] = direction_noise_;
  random_direction[2] = direction_noise_;
  random_direction = random_direction.CalcNormalizedVector();

  double error_angle_rad = angle_noise_;
  libra::Quaternion error_quaternion(random_direction, error_angle_rad);

  // Get true value
  measured_quaternion_rb2tb_ = rel_info_.GetRelativeAttitudeQuaternion(target_sat_id_, reference_sat_id_);
  measured_quaternion_rb2tb_ = error_quaternion * measured_quaternion_rb2tb_;
  measured_euler_angle_rb2tb_rad_ = measured_quaternion_rb2tb_.ConvertToEuler();
}

std::string RelativeAttitudeSensor::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeAttitudeSensor_";

  const std::string frame_name = std::to_string(reference_sat_id_) + "to" + std::to_string(target_sat_id_);
  str_tmp += WriteQuaternion(head + "quaternion", frame_name);
  str_tmp += WriteVector(head + "euler_angle", frame_name, "rad", 3);

  return str_tmp;
}

std::string RelativeAttitudeSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteQuaternion(measured_quaternion_rb2tb_);
  str_tmp += WriteVector(measured_euler_angle_rb2tb_rad_);

  return str_tmp;
}

RelativeAttitudeSensor InitializeRelativeAttitudeSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);
  char section[30] = "RELATIVE_ATTITUDE_SENSOR";

  // CompoBase
  int prescaler = ini_file.ReadInt(section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  double error_angle_standard_deviation_deg = ini_file.ReadDouble(section, "error_angle_standard_deviation_deg");
  double error_angle_standard_deviation_rad = libra::deg_to_rad * error_angle_standard_deviation_deg;

  // RelativeAttitudeSensor
  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }

  RelativeAttitudeSensor relative_attitude_sensor(prescaler, clock_gen, target_sat_id, reference_sat_id, rel_info,
                                                  error_angle_standard_deviation_rad);

  return relative_attitude_sensor;
}
