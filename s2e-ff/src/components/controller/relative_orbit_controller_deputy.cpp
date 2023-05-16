#include "relative_orbit_controller_deputy.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/math/constants.hpp>

RelativeOrbitControllerDeputy::RelativeOrbitControllerDeputy(const int prescaler, ClockGenerator* clock_gen, const int sc_id,
                                                             FfComponents2& components)
    : Component(prescaler, clock_gen), components_(components), sc_id_(sc_id) {
  mu_m3_s2_ = environment::earth_gravitational_constant_m3_s2;

  // TODO: set target
  double d_lambda = 0.00000433;  // 0.00000289;
  double d_ix = 0.00000001;
  double d_iy = 0.00000076;      // 0.00000051;

  Vector<6> target_roe{0.0};
  if (sc_id_ == 1) {  // deputy-1
    target_roe[1] = -d_lambda;
    target_roe[4] = -d_ix;
    target_roe[5] = d_iy;
  } else {  // deputy-2
    target_roe[1] = d_lambda;
    target_roe[4] = d_ix;
    target_roe[5] = -d_iy;
  }
  target_qns_roe_ = QuasiNonsingularRelativeOrbitalElements(a_m_, target_roe);
}

RelativeOrbitControllerDeputy::~RelativeOrbitControllerDeputy() {}

void RelativeOrbitControllerDeputy::MainRoutine(int count) {
  UNUSED(count);

  EstimateStates();
  QuasiNonsingularRelativeOrbitalElements diff_qns_roe = target_qns_roe_ - estimated_qns_roe_;

  if (count > 1840 && count < 2000) {  // FIXME: set maneuver timing
    // Calc Maneuver output
    dv_rtn_m_s_ = DoubleImpulse_seirios(dv_start_s_, dv_timing_s_, diff_qns_roe);
  }

  // Generate First impulse
  int first_start_timing = 3000 + (int)(dv_start_s_ / component_update_sec_);
  if (count > first_start_timing && (first_thrust_done_ == false)) {
    libra::Vector<3> f_rtn_N = (mass_kg_ / impulse_output_duration_sec_) * dv_rtn_m_s_;
    if (sc_id_ == 2) f_rtn_N[2] = -f_rtn_N[2];  // FIXME

    int finish_timing = first_start_timing + int(impulse_output_duration_sec_ / component_update_sec_);
    if (count > finish_timing) {
      first_thrust_done_ = true;
      f_rtn_N = libra::Vector<3>{0.0};
    }
    if (enable_thruster_) components_.GetForceGenerator().SetForce_rtn_N(f_rtn_N);
  }

  // Generate Second impulse
  int second_start_timing = first_start_timing + (int)(dv_timing_s_ / component_update_sec_);
  if (count > second_start_timing && (second_thrust_done_ == false)) {
    libra::Vector<3> f_rtn_N = (mass_kg_ / impulse_output_duration_sec_) * dv_rtn_m_s_;
    if (sc_id_ == 1) f_rtn_N[2] = -f_rtn_N[2];  // FIXME

    int finish_timing = second_start_timing + int(impulse_output_duration_sec_ / component_update_sec_);
    if (count > finish_timing) {
      second_thrust_done_ = true;
      f_rtn_N = libra::Vector<3>{0.0};
    }
    if (enable_thruster_) components_.GetForceGenerator().SetForce_rtn_N(f_rtn_N);
  }
}

std::string RelativeOrbitControllerDeputy::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeOrbitControllerDeputy_";

  str_tmp += WriteVector(head + "roe_est", "-", "-", 6);
  str_tmp += WriteScalar(head + "rel_dist_est", "m");
  str_tmp += WriteScalar(head + "inc_vector_angle_est", "rad");
  str_tmp += WriteVector(head + "dv_rtn", "rtn", "m/s", 3);
  str_tmp += WriteScalar(head + "dv_timing", "rad");

  return str_tmp;
}

std::string RelativeOrbitControllerDeputy::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(estimated_qns_roe_.GetRelativeOrbitalElementsAsVector());
  str_tmp += WriteScalar(estimated_relative_distance_m_);
  str_tmp += WriteScalar(estimated_inc_vec_angle_);
  str_tmp += WriteVector(dv_rtn_m_s_);
  str_tmp += WriteScalar(dv_timing_rad_);

  return str_tmp;
}

void RelativeOrbitControllerDeputy::EstimateStates() {
  a_m_ = 6928000.0;  // TODO: measure the latest semi major axis
  libra::Vector<3> measured_rel_pos_rtn_m = components_.GetRelativePositionSensor().GetMeasuredTargetPosition_rtn_m();
  libra::Vector<3> measured_rel_vel_rtn_m_s = components_.GetRelativeVelocitySensor().GetMeasuredTargetVelocity_rtn_m_s();
  estimated_qns_roe_ = QuasiNonsingularRelativeOrbitalElements(a_m_, measured_rel_pos_rtn_m, measured_rel_vel_rtn_m_s, mu_m3_s2_);
  // TODO: Averaging, Filtering

  // Others
  estimated_relative_distance_m_ = estimated_qns_roe_.GetDeltaMeanLongitude() * a_m_;
  estimated_inc_vec_angle_ = atan2(estimated_qns_roe_.GetDeltaInclinationY(), estimated_qns_roe_.GetDeltaInclinationX());
}

libra::Vector<3> RelativeOrbitControllerDeputy::InPlaneSingleImpulse(double& maneuver_timing_rad,
                                                                     QuasiNonsingularRelativeOrbitalElements diff_qns_roe) {
  // Reference information
  const double a = diff_qns_roe.GetReferenceSemiMajor_m();
  const double n = sqrt(mu_m3_s2_ / pow(a, 3.0));
  // Relative information
  const double d_a = diff_qns_roe.GetDeltaSemiMajor();
  const double d_ex = diff_qns_roe.GetDeltaEccentricityX();
  const double d_ey = diff_qns_roe.GetDeltaEccentricityY();
  const double d_e = sqrt(d_ex * d_ex + d_ey * d_ey);

  // Control output
  libra::Vector<3> dv_rtn_N{0.0};
  dv_rtn_N[0] = a * n * sqrt(d_e * d_e - d_a * d_a);
  dv_rtn_N[1] = a * n * d_a / 2.0;
  maneuver_timing_rad = atan2(dv_rtn_N[0], 2.0 * dv_rtn_N[1]) - atan2(d_ex, d_ey);

  return dv_rtn_N;
}

libra::Vector<3> RelativeOrbitControllerDeputy::OutPlaneSingleImpulse(double& maneuver_timing_rad,
                                                                      QuasiNonsingularRelativeOrbitalElements diff_qns_roe) {
  // Reference information
  const double a = diff_qns_roe.GetReferenceSemiMajor_m();
  const double n = sqrt(mu_m3_s2_ / pow(a, 3.0));
  // Relative information
  const double d_ix = diff_qns_roe.GetDeltaInclinationX();
  const double d_iy = diff_qns_roe.GetDeltaInclinationY();
  const double d_i = sqrt(d_ix * d_ix + d_iy * d_iy);

  // Control output
  libra::Vector<3> dv_rtn_N{0.0};
  dv_rtn_N[2] = a * n * d_i;
  maneuver_timing_rad = atan2(d_iy, d_ix);

  return dv_rtn_N;
}

libra::Vector<3> RelativeOrbitControllerDeputy::DoubleImpulse_seirios(double& first_maneuver_s, double& delta_maneuver_s,
                                                                      QuasiNonsingularRelativeOrbitalElements diff_qns_roe) {
  // Reference information
  const double a = diff_qns_roe.GetReferenceSemiMajor_m();
  const double n = sqrt(mu_m3_s2_ / pow(a, 3.0));
  // Relative information
  const double d_lambda = diff_qns_roe.GetDeltaMeanLongitude();
  const double d_ix = diff_qns_roe.GetDeltaInclinationX();
  const double d_iy = diff_qns_roe.GetDeltaInclinationY();
  const double d_i = sqrt(d_ix * d_ix + d_iy * d_iy);

  // Control output
  libra::Vector<3> dv_rtn_N{0.0};
  dv_rtn_N[0] = a * n * d_lambda / 4.0;
  dv_rtn_N[2] = a * n * d_i / 2.0;

  // Timing
  double first_maneuver_rad = atan2(d_iy, d_ix);
  double delta_maneuver_rad = libra::pi;
  first_maneuver_s = abs(first_maneuver_rad) / n;
  delta_maneuver_s = abs(delta_maneuver_rad) / n;

  return dv_rtn_N;
}
