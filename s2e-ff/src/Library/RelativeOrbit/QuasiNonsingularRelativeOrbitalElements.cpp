#include "QuasiNonsingularRelativeOrbitalElements.hpp"

#include <cmath>

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target) {
  double diff_raan = qns_oe_target.GetRaan_rad() - qns_oe_reference.GetRaan_rad();

  semi_major_axis_ref_m_ = qns_oe_reference.GetSemiMajor_m();

  d_semi_major_axis_ = (qns_oe_target.GetSemiMajor_m() - semi_major_axis_ref_m_) / semi_major_axis_ref_m_;
  d_mean_longitude_ = (qns_oe_target.GetMeanArgLatEpoch_rad() - qns_oe_reference.GetMeanArgLatEpoch_rad()) + diff_raan * cos(qns_oe_reference.GetInclination_rad());

  d_eccentricity_x_ = qns_oe_target.GetEccentricityX() - qns_oe_reference.GetEccentricityX();
  d_eccentricity_y_ = qns_oe_target.GetEccentricityY() - qns_oe_reference.GetEccentricityY();
  d_inclination_x_ = qns_oe_target.GetInclination_rad() - qns_oe_reference.GetInclination_rad();
  d_inclination_y_ = diff_raan * sin(qns_oe_reference.GetInclination_rad());
}

QuasiNonsingularRelativeOrbitalElements::~QuasiNonsingularRelativeOrbitalElements() {}

libra::Vector<3> QuasiNonsingularRelativeOrbitalElements::CalcRelativePositionCircularApprox_rtn_m(const double arg_lat_rad) {
  libra::Vector<3> relative_position_rtn_m;
  double cos_u = cos(arg_lat_rad);
  double sin_u = sin(arg_lat_rad);

  relative_position_rtn_m[0] = d_semi_major_axis_ - (d_eccentricity_x_ * cos_u + d_eccentricity_y_ * sin_u);
  relative_position_rtn_m[1] = -1.5 * d_semi_major_axis_ * arg_lat_rad + d_mean_longitude_ + 2.0 * (d_eccentricity_x_ * sin_u - d_eccentricity_y_ * cos_u);
  relative_position_rtn_m[2] = d_inclination_x_ * sin_u - d_inclination_y_ * cos_u;

  relative_position_rtn_m *= semi_major_axis_ref_m_;
  return relative_position_rtn_m;
}
