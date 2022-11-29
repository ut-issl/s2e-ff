/**
 * @file QuasiNonsingularOrbitalElementDifferences.cpp
 * @brief Orbital elements differences to avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularOrbitalElementDifferences.hpp"

#include <cmath>

QuasiNonsingularOrbitalElementDifferences::QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                                                                     const QuasiNonsingularOrbitalElements qns_oe_target) {
  semi_major_axis_ref_m_ = qns_oe_reference.GetSemiMajor_m();

  diff_qns_oe_ = qns_oe_target - qns_oe_reference;
}

QuasiNonsingularOrbitalElementDifferences::~QuasiNonsingularOrbitalElementDifferences() {}

libra::Vector<3> QuasiNonsingularOrbitalElementDifferences::CalcRelativePositionCircularApprox_rtn_m(const double true_anomaly_rad) {
  libra::Vector<3> relative_position_rtn_m;
  double cos_f = cos(true_anomaly_rad);
  double sin_f = sin(true_anomaly_rad);

  return relative_position_rtn_m;
}
