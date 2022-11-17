#include "QuasiNonsingularOrbitalElementDifferences.hpp"

#include <cmath>

QuasiNonsingularOrbitalElementDifferences::QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target) {
  semi_major_axis_ref_m_ = qns_oe_reference.GetSemiMajor_m();

  d_semi_major_axis_m_ = qns_oe_target.GetSemiMajor_m() - qns_oe_reference.GetSemiMajor_m();
  d_eccentricity_x_ = qns_oe_target.GetEccentricityX() - qns_oe_reference.GetEccentricityX();
  d_eccentricity_y_ = qns_oe_target.GetEccentricityY() - qns_oe_reference.GetEccentricityY();
  d_inclination_rad_ = qns_oe_target.GetInclination_rad() - qns_oe_reference.GetInclination_rad();
  d_raan_rad_ = qns_oe_target.GetRaan_rad() - qns_oe_reference.GetRaan_rad();
  d_mean_arg_latitude_epoch_rad_ = qns_oe_target.GetMeanArgLatEpoch_rad() - qns_oe_reference.GetMeanArgLatEpoch_rad();
}

QuasiNonsingularOrbitalElementDifferences::~QuasiNonsingularOrbitalElementDifferences() {}

libra::Vector<3> QuasiNonsingularOrbitalElementDifferences::CalcRelativePositionCircularApprox_rtn_m(const double true_anomaly_rad)
{
  libra::Vector<3> relative_position_rtn_m;
  double cos_f = cos(true_anomaly_rad);
  double sin_f = sin(true_anomaly_rad);


}
