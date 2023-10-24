/**
 * @file relative_velocity_sensor.cpp
 * @brief Relative velocity sensor
 */

#include "relative_velocity_sensor.hpp"

RelativeVelocitySensor::RelativeVelocitySensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id,
                                               const int reference_sat_id, const RelativeVelocitySensorErrorFrame error_frame,
                                               const RelativeInformation& rel_info, const Dynamics& dynamics)
    : Component(prescaler, clock_gen),
      Sensor(sensor_base),
      target_sat_id_(target_sat_id),
      reference_sat_id_(reference_sat_id),
      error_frame_(error_frame),
      rel_info_(rel_info),
      dynamics_(dynamics) {}

RelativeVelocitySensor::~RelativeVelocitySensor() {}

void RelativeVelocitySensor::MainRoutine(int count) {
  UNUSED(count);

  // Get true value
  libra::Vector<3> true_target_velocity_i_m_s = rel_info_.GetRelativeVelocity_i_m_s(target_sat_id_, reference_sat_id_);
  libra::Vector<3> true_target_velocity_rtn_m_s = rel_info_.GetRelativeVelocity_rtn_m_s(target_sat_id_, reference_sat_id_);

  // Add noise at body frame and frame conversion
  libra::Quaternion q_i2rtn = dynamics_.GetOrbit().CalcQuaternion_i2lvlh();
  switch (error_frame_) {
    case RelativeVelocitySensorErrorFrame::INERTIAL: {
      measured_target_velocity_i_m_s_ = Measure(true_target_velocity_i_m_s);
      libra::Vector<3> d_vel_i_m_s = measured_target_velocity_i_m_s_ - true_target_velocity_i_m_s;
      // Frame conversion
      libra::Vector<3> d_vel_rtn_m_s = q_i2rtn.FrameConversion(d_vel_i_m_s);
      measured_target_velocity_rtn_m_s_ = true_target_velocity_rtn_m_s + d_vel_rtn_m_s;
      break;
    }
    case RelativeVelocitySensorErrorFrame::RTN: {
      measured_target_velocity_rtn_m_s_ = Measure(true_target_velocity_rtn_m_s);
      libra::Vector<3> d_vel_rtn_m_s = measured_target_velocity_rtn_m_s_ - true_target_velocity_rtn_m_s;
      // Frame conversion
      libra::Vector<3> d_vel_i_m_s = q_i2rtn.InverseFrameConversion(d_vel_rtn_m_s);
      measured_target_velocity_i_m_s_ = true_target_velocity_i_m_s + d_vel_i_m_s;
      break;
    }
    default:
      break;
  }
}

std::string RelativeVelocitySensor::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeVelocitySensor_";
  str_tmp += WriteVector(head + "velocity", "i", "m", 3);
  str_tmp += WriteVector(head + "velocity", "rtn", "m", 3);

  return str_tmp;
}

std::string RelativeVelocitySensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(measured_target_velocity_i_m_s_);
  str_tmp += WriteVector(measured_target_velocity_rtn_m_s_);

  return str_tmp;
}
