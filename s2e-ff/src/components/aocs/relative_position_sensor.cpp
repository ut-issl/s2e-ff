/**
 * @file relative_position_sensor.cpp
 * @brief Relative position sensor
 */

#include "relative_position_sensor.hpp"

#include <components/base/sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativePositionSensor::RelativePositionSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id,
                                               const int reference_sat_id, const RelativePositionSensorErrorFrame error_frame,
                                               const RelativeInformation& rel_info, const Dynamics& dynamics)
    : Component(prescaler, clock_gen),
      Sensor(sensor_base),
      target_sat_id_(target_sat_id),
      reference_sat_id_(reference_sat_id),
      error_frame_(error_frame),
      rel_info_(rel_info),
      dynamics_(dynamics) {}

RelativePositionSensor::~RelativePositionSensor() {}

void RelativePositionSensor::MainRoutine(int count) {
  UNUSED(count);

  // Get true value
  measured_target_position_i_m_ = rel_info_.GetRelativePosition_i_m(target_sat_id_, reference_sat_id_);
  measured_target_position_rtn_m_ = rel_info_.GetRelativePosition_rtn_m(target_sat_id_, reference_sat_id_);
  libra::Quaternion q_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
  measured_target_position_body_m_ = q_i2b.FrameConversion(measured_target_position_i_m_);

  // Add noise at body frame and frame conversion
  libra::Quaternion q_i2rtn = dynamics_.GetOrbit().CalcQuaternion_i2lvlh();
  switch (error_frame_) {
    case RelativePositionSensorErrorFrame::INERTIAL:
      measured_target_position_i_m_ = Measure(measured_target_position_i_m_);
      // Frame conversion
      measured_target_position_rtn_m_ = q_i2rtn.FrameConversion(measured_target_position_i_m_);
      measured_target_position_body_m_ = q_i2b.FrameConversion(measured_target_position_i_m_);
      break;
    case RelativePositionSensorErrorFrame::RTN:
      measured_target_position_rtn_m_ = Measure(measured_target_position_rtn_m_);
      // Frame conversion
      measured_target_position_i_m_ = q_i2rtn.InverseFrameConversion(measured_target_position_rtn_m_);
      measured_target_position_body_m_ = q_i2b.FrameConversion(measured_target_position_i_m_);
      break;
    case RelativePositionSensorErrorFrame::BODY:
      measured_target_position_body_m_ = Measure(measured_target_position_body_m_);
      // Frame conversion
      measured_target_position_i_m_ = q_i2b.InverseFrameConversion(measured_target_position_body_m_);
      measured_target_position_rtn_m_ = q_i2rtn.FrameConversion(measured_target_position_i_m_);
      break;
    default:
      break;
  }
}

std::string RelativePositionSensor::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativePositionSensor_";
  str_tmp += WriteVector(head + "position", "i", "m", 3);
  str_tmp += WriteVector(head + "position", "rtn", "m", 3);
  str_tmp += WriteVector(head + "position", "b", "m", 3);

  return str_tmp;
}

std::string RelativePositionSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(measured_target_position_i_m_);
  str_tmp += WriteVector(measured_target_position_rtn_m_);
  str_tmp += WriteVector(measured_target_position_body_m_);

  return str_tmp;
}

RelativePositionSensor InitializeRelativePositionSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics,
                                                        const int reference_sat_id_input) {
  // General
  IniAccess ini_file(file_name);
  char section[30] = "RELATIVE_POSITION_SENSOR";

  // CompoBase
  int prescaler = ini_file.ReadInt(section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // RelativePositionSensor
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

  // SensorBase
  Sensor<3> sensor_base = ReadSensorInformation<3>(file_name, compo_step_time_s * (double)(prescaler), section, "m");

  RelativePositionSensor relative_position_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, error_frame, rel_info,
                                                  dynamics);

  return relative_position_sensor;
}
