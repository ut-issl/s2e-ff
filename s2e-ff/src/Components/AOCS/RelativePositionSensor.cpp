#include "RelativePositionSensor.hpp"

RelativePositionSensor::RelativePositionSensor(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base, const int target_sat_id,
                                               const int reference_sat_id, const RelativeInformation& rel_info, const Dynamics& dynamics)
    : ComponentBase(prescaler, clock_gen),
      SensorBase(sensor_base),
      target_sat_id_(target_sat_id),
      reference_sat_id_(reference_sat_id),
      rel_info_(rel_info),
      dynamics_(dynamics) {}

RelativePositionSensor::~RelativePositionSensor() {}

void RelativePositionSensor::MainRoutine(int count) {
  UNUSED(count);

  measured_target_position_i_m_ = rel_info_.GetRelativePosition_i_m(target_sat_id_, reference_sat_id_);
  measured_target_position_rtn_m_ = rel_info_.GetRelativePosition_rtn_m(target_sat_id_, reference_sat_id_);

  libra::Quaternion q_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
  measured_target_position_body_m_ = q_i2b.frame_conv(measured_target_position_i_m_);
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
