#include "RelativeOrbitalElements.hpp"

#include <cmath>

RelativeOrbitalElements::RelativeOrbitalElements(const NonsingularOrbitalElements noe_reference, const NonsingularOrbitalElements noe_target) {
  double diff_raan = noe_target.GetRaan() - noe_reference.GetRaan();

  a_ref_m_ = noe_reference.GetSemiMajor();
  delta_a_ = (noe_target.GetSemiMajor() - a_ref_m_) / a_ref_m_;
  delta_lambda_ = (noe_target.GetArgLonEpoch() - noe_reference.GetArgLonEpoch()) + diff_raan * cos(noe_reference.GetInclination());

  delta_e_x_ = noe_target.GetEccentricityX() - noe_reference.GetEccentricityX();
  delta_e_y_ = noe_target.GetEccentricityY() - noe_reference.GetEccentricityY();
  delta_i_x_ = noe_target.GetInclination() - noe_reference.GetInclination();
  delta_i_y_ = diff_raan * sin(noe_reference.GetInclination());
}

RelativeOrbitalElements::~RelativeOrbitalElements() {}

libra::Vector<3> RelativeOrbitalElements::CalcRelativePositionRtn_m(const double arg_lat_rad) {
  libra::Vector<3> relative_position_rtn_m;
  double cos_u = cos(arg_lat_rad);
  double sin_u = sin(arg_lat_rad);

  relative_position_rtn_m[0] = delta_a_ - (delta_e_x_ * cos_u + delta_e_y_ * sin_u);
  relative_position_rtn_m[1] = -1.5 * delta_a_ * arg_lat_rad + delta_lambda_ + 2.0 * (delta_e_x_ * sin_u - delta_e_y_ * cos_u);
  relative_position_rtn_m[2] = delta_i_x_ * sin_u - delta_i_y_ * cos_u;

  relative_position_rtn_m *= a_ref_m_;
  return relative_position_rtn_m;
}
