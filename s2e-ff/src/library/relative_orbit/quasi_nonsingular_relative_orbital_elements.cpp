/**
 * @file QuasiNonsingularRelativeOrbitalElements.cpp
 * @brief Relative orbital elements defined by eccentricity/inclination vectors to avoid singularity when the eccentricity is near zero.
 */

#include "quasi_nonsingular_relative_orbital_elements.hpp"

#include <cmath>

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements() {
  semi_major_axis_ref_m_ = 0.0;

  d_semi_major_axis_ = 0.0;
  d_mean_longitude_ = 0.0;
  d_eccentricity_x_ = 0.0;
  d_eccentricity_y_ = 0.0;
  d_inclination_x_ = 0.0;
  d_inclination_y_ = 0.0;
}

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements(const double semi_major_axis_ref_m,
                                                                                 const libra::Vector<6> roe_as_vector)
    : semi_major_axis_ref_m_(semi_major_axis_ref_m),
      d_semi_major_axis_(roe_as_vector[0]),
      d_mean_longitude_(roe_as_vector[1]),
      d_eccentricity_x_(roe_as_vector[2]),
      d_eccentricity_y_(roe_as_vector[3]),
      d_inclination_x_(roe_as_vector[4]),
      d_inclination_y_(roe_as_vector[5]) {}

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                                                                 const QuasiNonsingularOrbitalElements qns_oe_target) {
  semi_major_axis_ref_m_ = qns_oe_reference.GetSemiMajor_m();

  d_semi_major_axis_ = (qns_oe_target.GetSemiMajor_m() - semi_major_axis_ref_m_) / semi_major_axis_ref_m_;
  d_eccentricity_x_ = qns_oe_target.GetEccentricityX() - qns_oe_reference.GetEccentricityX();
  d_eccentricity_y_ = qns_oe_target.GetEccentricityY() - qns_oe_reference.GetEccentricityY();
  d_inclination_x_ = qns_oe_target.GetInclination_rad() - qns_oe_reference.GetInclination_rad();

  const double diff_raan = qns_oe_target.GetRaan_rad() - qns_oe_reference.GetRaan_rad();
  d_inclination_y_ = diff_raan * sin(qns_oe_reference.GetInclination_rad());

  // Calc difference of argument of latitude
  const double d_mean_arg_lat_rad = CalcDiffMeanArgLat_rad(qns_oe_reference, qns_oe_target);

  d_mean_longitude_ = d_mean_arg_lat_rad + diff_raan * cos(qns_oe_reference.GetInclination_rad());
}

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements(const double semi_major_axis_ref_m,
                                                                                 const libra::Vector<3> relative_position_rtn_m,
                                                                                 const libra::Vector<3> relative_velocity_rtn_m_s,
                                                                                 const double mu_m3_s2) {
  const double a = semi_major_axis_ref_m;
  const double n = sqrt(mu_m3_s2 / (a * a * a));
  const libra::Vector<3> dv = (1.0 / n) * relative_velocity_rtn_m_s;

  // Relative info
  d_semi_major_axis_ = (4.0 * relative_position_rtn_m[0] + 2.0 * dv[1]) / a;
  d_mean_longitude_ = (relative_position_rtn_m[1] - 2.0 * dv[0]) / a;
  d_eccentricity_x_ = (3.0 * relative_position_rtn_m[0] + 2.0 * dv[1]) / a;
  d_eccentricity_y_ = (-1.0 * dv[0]) / a;
  d_inclination_x_ = (dv[2]) / a;
  d_inclination_y_ = (-1.0 * relative_position_rtn_m[2]) / a;

  // Reference info
  semi_major_axis_ref_m_ = a;
}

QuasiNonsingularRelativeOrbitalElements::~QuasiNonsingularRelativeOrbitalElements() {}

libra::Vector<3> QuasiNonsingularRelativeOrbitalElements::CalcRelativePositionCircularApprox_rtn_m(const double mean_arg_lat_rad) {
  libra::Vector<3> relative_position_rtn_m;
  const double cos_u = cos(mean_arg_lat_rad);
  const double sin_u = sin(mean_arg_lat_rad);

  relative_position_rtn_m[0] = d_semi_major_axis_ - (d_eccentricity_x_ * cos_u + d_eccentricity_y_ * sin_u);
  relative_position_rtn_m[1] =
      -1.5 * d_semi_major_axis_ * mean_arg_lat_rad + d_mean_longitude_ + 2.0 * (d_eccentricity_x_ * sin_u - d_eccentricity_y_ * cos_u);
  relative_position_rtn_m[2] = d_inclination_x_ * sin_u - d_inclination_y_ * cos_u;

  relative_position_rtn_m *= semi_major_axis_ref_m_;
  return relative_position_rtn_m;
}

libra::Vector<3> QuasiNonsingularRelativeOrbitalElements::CalcRelativeVelocityCircularApprox_rtn_m_s(const double mean_arg_lat_rad,
                                                                                                     const double mu_m3_s2) {
  libra::Vector<3> relative_velocity_rtn_m_s;
  const double cos_u = cos(mean_arg_lat_rad);
  const double sin_u = sin(mean_arg_lat_rad);

  const double a = semi_major_axis_ref_m_;
  const double n = sqrt(mu_m3_s2 / (a * a * a));

  relative_velocity_rtn_m_s[0] = d_eccentricity_x_ * sin_u - d_eccentricity_y_ * cos_u;
  relative_velocity_rtn_m_s[1] = -1.5 * d_semi_major_axis_ + 2.0 * (d_eccentricity_x_ * cos_u + d_eccentricity_y_ * sin_u);
  relative_velocity_rtn_m_s[2] = d_inclination_x_ * cos_u + d_inclination_y_ * sin_u;

  relative_velocity_rtn_m_s *= (semi_major_axis_ref_m_ * n);
  return relative_velocity_rtn_m_s;
}

double QuasiNonsingularRelativeOrbitalElements::CalcDiffMeanArgLat_rad(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                                                       const QuasiNonsingularOrbitalElements qns_oe_target) {
  // Reference info
  const double q1 = qns_oe_reference.GetEccentricityX();
  const double q2 = qns_oe_reference.GetEccentricityY();
  const double e2 = q1 * q1 + q2 * q2;
  const double e = sqrt(e2);
  const double eta2 = 1.0 - e2;
  const double eta = sqrt(eta2);

  const double cos_theta = cos(qns_oe_reference.GetTrueLatAng_rad());
  const double sin_theta = sin(qns_oe_reference.GetTrueLatAng_rad());
  const double e_cos_f = q1 * cos_theta + q2 * sin_theta;
  const double denominator = (1.0 + e_cos_f) * (1.0 + e_cos_f);

  const double arg_peri_ref_rad = atan2(qns_oe_reference.GetEccentricityY(), qns_oe_reference.GetEccentricityX());
  const double true_anomaly_ref_rad = qns_oe_reference.GetTrueLatAng_rad() - arg_peri_ref_rad;
  const double sin_f = sin(true_anomaly_ref_rad);

  // Difference Info
  const double arg_peri_target_rad = atan2(qns_oe_target.GetEccentricityY(), qns_oe_target.GetEccentricityX());
  const double true_anomaly_target_rad = qns_oe_target.GetTrueLatAng_rad() - arg_peri_target_rad;
  const double d_true_anomaly_rad = true_anomaly_target_rad - true_anomaly_ref_rad;

  const double q1_target = qns_oe_reference.GetEccentricityX();
  const double q2_target = qns_oe_reference.GetEccentricityY();
  const double e_target = sqrt(q1_target * q1_target + q2_target * q2_target);
  const double d_e = e_target - e;

  const double d_mean_arg_lat_rad = (eta / denominator) * (eta2 * d_true_anomaly_rad - sin_f * (2.0 + e_cos_f) * d_e);
  return (arg_peri_target_rad - arg_peri_ref_rad) + d_mean_arg_lat_rad;
}

QuasiNonsingularRelativeOrbitalElements operator-(const QuasiNonsingularRelativeOrbitalElements lhs,
                                                  const QuasiNonsingularRelativeOrbitalElements rhs) {
  libra::Vector<6> out_roe = lhs.GetRelativeOrbitalElementsAsVector() - rhs.GetRelativeOrbitalElementsAsVector();
  QuasiNonsingularRelativeOrbitalElements out(lhs.GetReferenceSemiMajor_m(), out_roe);

  return out;
}
