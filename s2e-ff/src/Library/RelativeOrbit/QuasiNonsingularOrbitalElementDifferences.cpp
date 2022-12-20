/**
 * @file QuasiNonsingularOrbitalElementDifferences.cpp
 * @brief Orbital elements differences to avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularOrbitalElementDifferences.hpp"

#include <cmath>

QuasiNonsingularOrbitalElementDifferences::QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                                                                     const QuasiNonsingularOrbitalElements qns_oe_target)
    : qns_oe_reference_(qns_oe_reference) {
  diff_qns_oe_ = qns_oe_target - qns_oe_reference;
}

QuasiNonsingularOrbitalElementDifferences::~QuasiNonsingularOrbitalElementDifferences() {}

libra::Vector<3> QuasiNonsingularOrbitalElementDifferences::CalcRelativePositionCircularApprox_rtn_m(const double true_anomaly_rad) {
  // Reference orbit variables
  const double a = qns_oe_reference_.GetSemiMajor_m();
  const double ex = qns_oe_reference_.GetEccentricityX();
  const double ey = qns_oe_reference_.GetEccentricityY();
  const double i = qns_oe_reference_.GetInclination_rad();
  const double theta = qns_oe_reference_.GetMeanArgLatEpoch_rad();  // + some thing?
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double p = a * (1.0 - (ex * ex + ey * ey));  //!< Semilatus rectum

  // Relative orbit variables
  const double d_a = diff_qns_oe_.GetSemiMajor_m();
  const double d_ex = diff_qns_oe_.GetEccentricityX();
  const double d_ey = diff_qns_oe_.GetEccentricityY();
  const double d_theta = diff_qns_oe_.GetMeanArgLatEpoch_rad();
  const double d_i = diff_qns_oe_.GetInclination_rad();
  const double d_raan = diff_qns_oe_.GetRaan_rad();

  // calculation
  const double r = p / (1.0 + ex * cos_theta + ey * sin_theta);
  const double v_r = (ex * sin_theta - ey * cos_theta);
  const double v_t = (1.0 + ex * cos_theta + ey * sin_theta);
  const double d_r = r / a * d_a + v_r / v_t * r * d_theta - r / p * ((2.0 * a * ex + r * cos_theta) * d_ex + (2.0 * a * ey + r * sin_theta) * d_ey);

  // Output
  libra::Vector<3> relative_position_rtn_m;
  relative_position_rtn_m[0] = d_r;
  relative_position_rtn_m[1] = r * (d_theta + d_raan * cos(i));
  relative_position_rtn_m[2] = r * (d_i * sin_theta - d_raan * cos_theta * sin(i));

  return relative_position_rtn_m;
}
