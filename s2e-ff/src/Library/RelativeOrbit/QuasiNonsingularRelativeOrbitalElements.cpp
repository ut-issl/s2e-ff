#include "QuasiNonsingularRelativeOrbitalElements.hpp"

#include <cmath>

QuasiNonsingularRelativeOrbitalElements::QuasiNonsingularRelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target) {
  double diff_raan = qns_oe_target.GetRaan() - qns_oe_reference.GetRaan();

  a_ref_m_ = qns_oe_reference.GetSemiMajor();
  delta_a_ = (qns_oe_target.GetSemiMajor() - a_ref_m_) / a_ref_m_;
  delta_lambda_ = (qns_oe_target.GetMeanArgLatEpoch() - qns_oe_reference.GetMeanArgLatEpoch()) + diff_raan * cos(qns_oe_reference.GetInclination());

  delta_e_x_ = qns_oe_target.GetEccentricityX() - qns_oe_reference.GetEccentricityX();
  delta_e_y_ = qns_oe_target.GetEccentricityY() - qns_oe_reference.GetEccentricityY();
  delta_i_x_ = qns_oe_target.GetInclination() - qns_oe_reference.GetInclination();
  delta_i_y_ = diff_raan * sin(qns_oe_reference.GetInclination());
}

QuasiNonsingularRelativeOrbitalElements::~QuasiNonsingularRelativeOrbitalElements() {}

libra::Vector<3> QuasiNonsingularRelativeOrbitalElements::CalcRelativePositionRtn_m(const double arg_lat_rad) {
  libra::Vector<3> relative_position_rtn_m;
  double cos_u = cos(arg_lat_rad);
  double sin_u = sin(arg_lat_rad);

  relative_position_rtn_m[0] = delta_a_ - (delta_e_x_ * cos_u + delta_e_y_ * sin_u);
  relative_position_rtn_m[1] = -1.5 * delta_a_ * arg_lat_rad + delta_lambda_ + 2.0 * (delta_e_x_ * sin_u - delta_e_y_ * cos_u);
  relative_position_rtn_m[2] = delta_i_x_ * sin_u - delta_i_y_ * cos_u;

  relative_position_rtn_m *= a_ref_m_;
  return relative_position_rtn_m;
}
