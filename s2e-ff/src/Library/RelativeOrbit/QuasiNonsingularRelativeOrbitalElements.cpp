/**
 * @file QuasiNonsingularRelativeOrbitalElements.cpp
 * @brief Relative orbital elements defined by eccentricity/inclination vectors to avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularRelativeOrbitalElements.hpp"

#include <cmath>

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

QuasiNonsingularRelativeOrbitalElements::~QuasiNonsingularRelativeOrbitalElements() {}

libra::Vector<3> QuasiNonsingularRelativeOrbitalElements::CalcRelativePositionCircularApprox_rtn_m(const double arg_lat_rad) {
  libra::Vector<3> relative_position_rtn_m;
  double cos_u = cos(arg_lat_rad);
  double sin_u = sin(arg_lat_rad);

  relative_position_rtn_m[0] = d_semi_major_axis_ - (d_eccentricity_x_ * cos_u + d_eccentricity_y_ * sin_u);
  relative_position_rtn_m[1] =
      -1.5 * d_semi_major_axis_ * arg_lat_rad + d_mean_longitude_ + 2.0 * (d_eccentricity_x_ * sin_u - d_eccentricity_y_ * cos_u);
  relative_position_rtn_m[2] = d_inclination_x_ * sin_u - d_inclination_y_ * cos_u;

  relative_position_rtn_m *= semi_major_axis_ref_m_;
  return relative_position_rtn_m;
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
  const double sin_f = (q1 * sin_theta - q2 * cos_theta) / e;

  const double denominator = (1.0 + e_cos_f) * (1.0 + e_cos_f);

  // Difference Info
  const double arg_peri_ref_rad = atan2(qns_oe_reference.GetEccentricityY(), qns_oe_reference.GetEccentricityX());
  const double true_anomaly_ref_rad = qns_oe_reference.GetTrueLatAng_rad() - arg_peri_ref_rad;
  const double arg_peri_target_rad = atan2(qns_oe_target.GetEccentricityY(), qns_oe_target.GetEccentricityX());
  const double true_anomaly_target_rad = qns_oe_target.GetTrueLatAng_rad() - arg_peri_target_rad;
  const double d_true_anomaly_rad = true_anomaly_target_rad - true_anomaly_ref_rad;

  const double q1_target = qns_oe_reference.GetEccentricityX();
  const double q2_target = qns_oe_reference.GetEccentricityY();
  const double e_target = sqrt(q1_target * q1_target + q2_target * q2_target);

  const double d_eccentricity = e_target - e;

  const double d_mean_arg_lat_rad = (eta / denominator) * (eta2 * d_true_anomaly_rad - sin_f * (2.0 + e_cos_f) * d_eccentricity);
  return (arg_peri_target_rad - arg_peri_ref_rad) + d_mean_arg_lat_rad;
}
