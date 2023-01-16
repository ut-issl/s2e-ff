#include "RelativeOrbitController.hpp"

#include <Environment/Global/PhysicalConstants.hpp>

RelativeOrbitController::RelativeOrbitController(const int prescaler, ClockGenerator* clock_gen, FfComponents& components)
    : ComponentBase(prescaler, clock_gen), components_(components) {
  mu_m3_s2_ = environment::earth_gravitational_constant_m3_s2;
}

RelativeOrbitController::~RelativeOrbitController() {}

void RelativeOrbitController::MainRoutine(int count) {
  UNUSED(count);

  a_m_ = 6928000.0;  // TODO: measure the latest semi major axis
  libra::Vector<3> measured_rel_pos_rtn_m = components_.GetRelativePositionSensor().GetMeasuredTargetPosition_rtn_m();
  libra::Vector<3> measured_rel_vel_rtn_m_s = components_.GetRelativeVelocitySensor().GetMeasuredTargetVelocity_rtn_m_s();
  QuasiNonsingularRelativeOrbitalElements qns_roe(a_m_, measured_rel_pos_rtn_m, measured_rel_vel_rtn_m_s, mu_m3_s2_);

  double relative_distance_m = qns_roe.GetDeltaMeanLongitude() * a_m_;
}

std::string RelativeOrbitController::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string RelativeOrbitController::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
