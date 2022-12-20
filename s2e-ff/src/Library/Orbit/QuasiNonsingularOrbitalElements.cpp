/**
 * @file QuasiNonsingularOrbitalElements.cpp
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularOrbitalElements.hpp"

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements() {
  semi_major_axis_m_ = 0.0;
  true_latitude_angle_rad_ = 0.0;
  eccentricity_x_ = 0.0;
  eccentricity_y_ = 0.0;
  inclination_rad_ = 0.0;
  raan_rad_ = 0.0;
  CalcOrbitParameters();
}

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements(const double semi_major_axis_m, const double eccentricity_x,
                                                                 const double eccentricity_y, const double inclination_rad, const double raan_rad,
                                                                 const double mean_arg_latitude_rad)
    : semi_major_axis_m_(semi_major_axis_m),
      eccentricity_x_(eccentricity_x),
      eccentricity_y_(eccentricity_y),
      inclination_rad_(inclination_rad),
      raan_rad_(raan_rad),
      true_latitude_angle_rad_(mean_arg_latitude_rad) {
  CalcOrbitParameters();
}

QuasiNonsingularOrbitalElements ::~QuasiNonsingularOrbitalElements() {}

QuasiNonsingularOrbitalElements operator-(const QuasiNonsingularOrbitalElements lhs, const QuasiNonsingularOrbitalElements rhs) {
  double semi_major_axis_m = lhs.GetSemiMajor_m() - rhs.GetSemiMajor_m();
  double eccentricity_x = lhs.GetEccentricityX() - rhs.GetEccentricityX();
  double eccentricity_y = lhs.GetEccentricityY() - rhs.GetEccentricityY();
  double inclination_rad = lhs.GetInclination_rad() - rhs.GetInclination_rad();
  double raan_rad = lhs.GetRaan_rad() - rhs.GetRaan_rad();
  double true_latitude_angle_rad = lhs.GetTrueLatAng_rad() - rhs.GetTrueLatAng_rad();

  QuasiNonsingularOrbitalElements out(semi_major_axis_m, eccentricity_x, eccentricity_y, inclination_rad, raan_rad, true_latitude_angle_rad);

  return out;
}

void QuasiNonsingularOrbitalElements::CalcOrbitParameters() {
  semi_latus_rectum_m_ = semi_major_axis_m_ * (1.0 - (eccentricity_x_ * eccentricity_x_ + eccentricity_y_ * eccentricity_y_));
  radius_m_ = semi_latus_rectum_m_ / (1.0 + eccentricity_x_ * cos(true_latitude_angle_rad_) + eccentricity_y_ * sin(true_latitude_angle_rad_));
}
