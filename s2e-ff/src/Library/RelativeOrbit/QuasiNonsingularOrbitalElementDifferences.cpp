#include "QuasiNonsingularOrbitalElementDifferences.hpp"

#include <cmath>

QuasiNonsingularOrbitalElementDifferences::QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target) {
  d_semi_major_axis_m_ = qns_oe_target.GetSemiMajor() - qns_oe_reference.GetSemiMajor();
  d_eccentricity_x_ = qns_oe_target.GetEccentricityX() - qns_oe_reference.GetEccentricityX();
  d_eccentricity_y_ = qns_oe_target.GetEccentricityY() - qns_oe_reference.GetEccentricityY();
  d_inclination_rad_ = qns_oe_target.GetInclination() - qns_oe_reference.GetInclination();
  d_raan_rad_ = qns_oe_target.GetRaan() - qns_oe_reference.GetRaan();
  d_mean_arg_latitude_epoch_rad_ = qns_oe_target.GetMeanArgLatEpoch() - qns_oe_reference.GetMeanArgLatEpoch();
}

QuasiNonsingularOrbitalElementDifferences::~QuasiNonsingularOrbitalElementDifferences() {}
