#include "QuasiNonsingularRelativeOrbitalElements.hpp"

#include <cmath>

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target) {
  double diff_raan = qns_oe_target.GetRaan_rad() - qns_oe_reference.GetRaan_rad();

  semi_major_axis_ref_m_ = qns_oe_reference.GetSemiMajor_m();
  delta_semi_major_axis_ = (qns_oe_target.GetSemiMajor_m() - semi_major_axis_ref_m_) / semi_major_axis_ref_m_;
  delta_mean_longitude_ = (qns_oe_target.GetMeanArgLatEpoch_rad() - qns_oe_reference.GetMeanArgLatEpoch_rad()) + diff_raan * cos(qns_oe_reference.GetInclination_rad());

  delta_eccentricity_x_ = qns_oe_target.GetEccentricityX() - qns_oe_reference.GetEccentricityX();
  delta_eccentricity_y_ = qns_oe_target.GetEccentricityY() - qns_oe_reference.GetEccentricityY();
  delta_inclination_x_ = qns_oe_target.GetInclination_rad() - qns_oe_reference.GetInclination_rad();
  delta_inclination_y_ = diff_raan * sin(qns_oe_reference.GetInclination_rad());
}

QuasiNonsingularRelativeOrbitalElements::~QuasiNonsingularRelativeOrbitalElements() {}

libra::Vector<3> QuasiNonsingularRelativeOrbitalElements::CalcRelativePositionRtn_m(const double arg_lat_rad) {
  libra::Vector<3> relative_position_rtn_m;
  double cos_u = cos(arg_lat_rad);
  double sin_u = sin(arg_lat_rad);

  relative_position_rtn_m[0] = delta_semi_major_axis_ - (delta_eccentricity_x_ * cos_u + delta_eccentricity_y_ * sin_u);
  relative_position_rtn_m[1] = -1.5 * delta_semi_major_axis_ * arg_lat_rad + delta_mean_longitude_ + 2.0 * (delta_eccentricity_x_ * sin_u - delta_eccentricity_y_ * cos_u);
  relative_position_rtn_m[2] = delta_inclination_x_ * sin_u - delta_inclination_y_ * cos_u;

  relative_position_rtn_m *= semi_major_axis_ref_m_;
  return relative_position_rtn_m;
}
