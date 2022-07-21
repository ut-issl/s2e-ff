#include "RelativeDistanceSensor.hpp"

RelativeDistanceSensor::RelativeDistanceSensor(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base,
                                               const int target_sat_id, const int reference_sat_id,
                                               const RelativeInformation& rel_info)
    : ComponentBase(prescaler, clock_gen), SensorBase(sensor_base),
      target_sat_id_(target_sat_id), reference_sat_id_(reference_sat_id), rel_info_(rel_info) {}

RelativeDistanceSensor::~RelativeDistanceSensor() {}

void RelativeDistanceSensor::MainRoutine(int count) {
  UNUSED(count);

  measured_distance_bw_ref_target_m_[0] = rel_info_.GetRelativeDistance_m(target_sat_id_, reference_sat_id_);
  measured_distance_bw_ref_target_m_ = Measure(measured_distance_bw_ref_target_m_);  // Add Noise
}

std::string RelativeDistanceSensor::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar("relative_distance_sensor_ref_to_target", "m");

  return str_tmp;
}

std::string RelativeDistanceSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(measured_distance_bw_ref_target_m_[0]);

  return str_tmp;
}
