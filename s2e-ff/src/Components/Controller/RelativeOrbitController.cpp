#include "RelativeOrbitController.hpp"

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>

RelativeOrbitController::RelativeOrbitController(const int prescaler, ClockGenerator* clock_gen, FfComponents& components)
    : ComponentBase(prescaler, clock_gen), components_(components) {
  mu_m3_s2_ = environment::earth_gravitational_constant_m3_s2;
  a_m_ = 6928000.0;  // FIXME

  // TODO: set target
  Vector<6> target_roe;
  target_roe[0] = 0.0;
  target_roe[1] = -0.00000289;
  target_roe[2] = 0.0;
  target_roe[3] = 0.0;
  target_roe[4] = 0.000000001;
  target_roe[5] = 0.0;
  target_qns_roe_ = QuasiNonsingularRelativeOrbitalElements(a_m_, target_roe);
}

RelativeOrbitController::~RelativeOrbitController() {}

void RelativeOrbitController::MainRoutine(int count) {
  UNUSED(count);
  EstimateStates();
  QuasiNonsingularRelativeOrbitalElements diff_qns_roe = target_qns_roe_ - estimated_qns_roe_;
  static double dv_start_s;
  static double dv_timing_s;
  if (count > 1840 && count < 2000) {  // FIXME: set maneuver timing
    dv_rtn_m_s_ = DoubleImpulse_seirios(dv_start_s, dv_timing_s, diff_qns_roe);
  }
  // Generate
  double compo_step_sec = 0.1;  // FIXME
  int first_start_timing = 3000 + (int)(dv_start_s / compo_step_sec);
  // First
  if (count > first_start_timing && (first_thrust_done_ == false)) {
    double mass_kg = 14.0;  // FIXME
    double output_sec = 10.0;
    libra::Vector<3> f_rtn_N = (mass_kg / output_sec) * dv_rtn_m_s_;
    int finish_timing = first_start_timing + int(output_sec / compo_step_sec);
    if (count > finish_timing) {
      first_thrust_done_ = true;
      f_rtn_N = libra::Vector<3>{0.0};
    }
    // components_.GetForceGenerator().SetForce_rtn_N(f_rtn_N);
  }
  // Second
  int second_start_timing = first_start_timing + (int)(dv_timing_s / compo_step_sec);
  if (count > second_start_timing && (second_thrust_done_ == false)) {
    double mass_kg = 14.0;        // FIXME
    double compo_step_sec = 0.1;  // FIXME
    double output_sec = 10.0;
    libra::Vector<3> f_rtn_N = (mass_kg / output_sec) * dv_rtn_m_s_;
    f_rtn_N[2] = -f_rtn_N[2];  // FIXME
    int finish_timing = second_start_timing + int(output_sec / compo_step_sec);
    if (count > finish_timing) {
      second_thrust_done_ = true;
      f_rtn_N = libra::Vector<3>{0.0};
    }
    // components_.GetForceGenerator().SetForce_rtn_N(f_rtn_N);
  }
}

std::string RelativeOrbitController::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeOrbitController_";

  str_tmp += WriteVector(head + "roe_est", "-", "-", 6);
  str_tmp += WriteScalar(head + "rel_dist_est", "m");
  str_tmp += WriteScalar(head + "inc_vector_angle_est", "rad");
  str_tmp += WriteVector(head + "dv_rtn", "rtn", "m/s", 3);
  str_tmp += WriteScalar(head + "dv_timing", "rad");

  return str_tmp;
}

std::string RelativeOrbitController::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(estimated_qns_roe_.GetRelativeOrbitalElementsAsVector());
  str_tmp += WriteScalar(estimated_relative_distance_m_);
  str_tmp += WriteScalar(estimated_inc_vec_angle_);
  str_tmp += WriteVector(dv_rtn_m_s_);
  str_tmp += WriteScalar(dv_timing_rad_);

  return str_tmp;
}

void RelativeOrbitController::EstimateStates() {
  a_m_ = 6928000.0;  // TODO: measure the latest semi major axis
  libra::Vector<3> measured_rel_pos_rtn_m = components_.GetRelativePositionSensor().GetMeasuredTargetPosition_rtn_m();
  libra::Vector<3> measured_rel_vel_rtn_m_s = components_.GetRelativeVelocitySensor().GetMeasuredTargetVelocity_rtn_m_s();
  estimated_qns_roe_ = QuasiNonsingularRelativeOrbitalElements(a_m_, measured_rel_pos_rtn_m, measured_rel_vel_rtn_m_s, mu_m3_s2_);
  // TODO: Averaging, Filtering

  // Others
  estimated_relative_distance_m_ = estimated_qns_roe_.GetDeltaMeanLongitude() * a_m_;
  estimated_inc_vec_angle_ = atan2(estimated_qns_roe_.GetDeltaInclinationY(), estimated_qns_roe_.GetDeltaInclinationX());
}

libra::Vector<3> RelativeOrbitController::InPlaneSingleImpulse(double& maneuver_timing_rad, QuasiNonsingularRelativeOrbitalElements diff_qns_roe) {
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

libra::Vector<3> RelativeOrbitController::OutPlaneSingleImpulse(double& maneuver_timing_rad, QuasiNonsingularRelativeOrbitalElements diff_qns_roe) {
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

libra::Vector<3> RelativeOrbitController::DoubleImpulse_seirios(double& first_maneuver_s, double& delta_maneuver_s,
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
  first_maneuver_s = first_maneuver_rad / n;
  delta_maneuver_s = delta_maneuver_rad / n;

  return dv_rtn_N;
}
