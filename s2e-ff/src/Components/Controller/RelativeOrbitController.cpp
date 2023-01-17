#include "RelativeOrbitController.hpp"

#include <Environment/Global/PhysicalConstants.hpp>

RelativeOrbitController::RelativeOrbitController(const int prescaler, ClockGenerator* clock_gen, FfComponents& components)
    : ComponentBase(prescaler, clock_gen), components_(components) {
  mu_m3_s2_ = environment::earth_gravitational_constant_m3_s2;
  a_m_ = 6928000.0;  // FIXME

  // TODO: set target
  Vector<6> target_roe;
  target_roe[0] = 0.0;
  target_roe[1] = 0.00000217;
  target_roe[2] = 0.0;
  target_roe[3] = 0.0;
  target_roe[4] = 0.0;
  target_roe[5] = 0.00000029;
  target_qns_roe_with_semi_major_ = QuasiNonsingularRelativeOrbitalElements(a_m_, target_roe);
}

RelativeOrbitController::~RelativeOrbitController() {}

void RelativeOrbitController::MainRoutine(int count) {
  UNUSED(count);
  EstimateStates();
}

std::string RelativeOrbitController::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeOrbitController_";

  str_tmp += WriteVector(head + "roe_est", "-", "-", 6);
  str_tmp += WriteScalar(head + "rel_dist_est", "m");

  return str_tmp;
}

std::string RelativeOrbitController::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(estimated_qns_roe_.GetRelativeOrbitalElementsAsVector());
  str_tmp += WriteScalar(estimated_relative_distance_m_);

  return str_tmp;
}

void RelativeOrbitController::EstimateStates() {
  a_m_ = 6928000.0;  // TODO: measure the latest semi major axis
  libra::Vector<3> measured_rel_pos_rtn_m = components_.GetRelativePositionSensor().GetMeasuredTargetPosition_rtn_m();
  libra::Vector<3> measured_rel_vel_rtn_m_s = components_.GetRelativeVelocitySensor().GetMeasuredTargetVelocity_rtn_m_s();
  estimated_qns_roe_ = QuasiNonsingularRelativeOrbitalElements(a_m_, measured_rel_pos_rtn_m, measured_rel_vel_rtn_m_s, mu_m3_s2_);
  // TODO: Averaging, Filtering

  estimated_relative_distance_m_ = estimated_qns_roe_.GetDeltaMeanLongitude() * a_m_;
}
