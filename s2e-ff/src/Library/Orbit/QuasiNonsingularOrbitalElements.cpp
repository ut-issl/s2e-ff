/**
 * @file QuasiNonsingularOrbitalElements.cpp
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularOrbitalElements.hpp"

#include <cfloat>

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements() {
  semi_major_axis_m_ = 0.0;
  true_latitude_angle_rad_ = 0.0;
  inclination_rad_ = 0.0;
  eccentricity_x_ = 0.0;
  eccentricity_y_ = 0.0;
  raan_rad_ = 0.0;
  CalcOrbitParameters();
}

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements(const double semi_major_axis_m, const double true_latitude_angle_rad,
                                                                 const double inclination_rad, const double eccentricity_x,
                                                                 const double eccentricity_y, const double raan_rad)
    : semi_major_axis_m_(semi_major_axis_m),
      true_latitude_angle_rad_(true_latitude_angle_rad),
      inclination_rad_(inclination_rad),
      eccentricity_x_(eccentricity_x),
      eccentricity_y_(eccentricity_y),
      raan_rad_(raan_rad) {
  CalcOrbitParameters();
}

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements(const double mu_m3_s2, const libra::Vector<3> position_i_m,
                                                                 const libra::Vector<3> velocity_i_m_s) {
  // common variables
  double r_m = norm(position_i_m);
  double v2_m2_s2 = inner_product(velocity_i_m_s, velocity_i_m_s);
  libra::Vector<3> h;
  h = outer_product(position_i_m, velocity_i_m_s);
  double h_norm = norm(h);

  // semi major axis
  semi_major_axis_m_ = mu_m3_s2 / (2.0 * mu_m3_s2 / r_m - v2_m2_s2);

  // inclination
  libra::Vector<3> h_direction = h;
  h_direction = normalize(h_direction);
  inclination_rad_ = acos(h_direction[2]);
  inclination_rad_ = libra::WrapTo2Pi(inclination_rad_);

  // RAAN
  double norm_h = sqrt(h[0] * h[0] + h[1] * h[1]);
  if (norm_h < 0.0 + DBL_EPSILON) {
    // We cannot define raan when i = 0
    raan_rad_ = 0.0;
  } else {
    raan_rad_ = asin(h[0] / sqrt(h[0] * h[0] + h[1] * h[1]));
  }
  raan_rad_ = libra::WrapTo2Pi(raan_rad_);

  // position in plane
  double x_p_m = position_i_m[0] * cos(raan_rad_) + position_i_m[1] * sin(raan_rad_);
  double tmp_m = -position_i_m[0] * sin(raan_rad_) + position_i_m[1] * cos(raan_rad_);
  double y_p_m = tmp_m * cos(inclination_rad_) + position_i_m[2] * sin(inclination_rad_);

  // velocity in plane
  double dx_p_m_s = velocity_i_m_s[0] * cos(raan_rad_) + velocity_i_m_s[1] * sin(raan_rad_);
  double dtmp_m_s = -velocity_i_m_s[0] * sin(raan_rad_) + velocity_i_m_s[1] * cos(raan_rad_);
  double dy_p_m_s = dtmp_m_s * cos(inclination_rad_) + velocity_i_m_s[2] * sin(inclination_rad_);

  // eccentricity
  eccentricity_x_ = (h_norm / mu_m3_s2) * dy_p_m_s - x_p_m / r_m;
  eccentricity_y_ = -(h_norm / mu_m3_s2) * dx_p_m_s - y_p_m / r_m;

  // true anomaly f_rad and eccentric anomaly u_rad
  true_latitude_angle_rad_ = atan2(y_p_m, x_p_m);
  true_latitude_angle_rad_ = libra::WrapTo2Pi(true_latitude_angle_rad_);

  CalcOrbitParameters();
}

QuasiNonsingularOrbitalElements ::~QuasiNonsingularOrbitalElements() {}

QuasiNonsingularOrbitalElements operator-(const QuasiNonsingularOrbitalElements lhs, const QuasiNonsingularOrbitalElements rhs) {
  double semi_major_axis_m = lhs.GetSemiMajor_m() - rhs.GetSemiMajor_m();
  double true_latitude_angle_rad = lhs.GetTrueLatAng_rad() - rhs.GetTrueLatAng_rad();
  double inclination_rad = lhs.GetInclination_rad() - rhs.GetInclination_rad();
  double eccentricity_x = lhs.GetEccentricityX() - rhs.GetEccentricityX();
  double eccentricity_y = lhs.GetEccentricityY() - rhs.GetEccentricityY();
  double raan_rad = lhs.GetRaan_rad() - rhs.GetRaan_rad();

  QuasiNonsingularOrbitalElements out(semi_major_axis_m, eccentricity_x, eccentricity_y, inclination_rad, raan_rad, true_latitude_angle_rad);

  return out;
}

void QuasiNonsingularOrbitalElements::CalcOrbitParameters() {
  semi_latus_rectum_m_ = semi_major_axis_m_ * (1.0 - (eccentricity_x_ * eccentricity_x_ + eccentricity_y_ * eccentricity_y_));
  radius_m_ = semi_latus_rectum_m_ / (1.0 + eccentricity_x_ * cos(true_latitude_angle_rad_) + eccentricity_y_ * sin(true_latitude_angle_rad_));
}
