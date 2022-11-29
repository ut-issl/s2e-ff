/**
 * @file QuasiNonsingularOrbitalElements.cpp
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularOrbitalElements.hpp"

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements() {
  semi_major_axis_m_ = 0.0;
  mean_arg_latitude_epoch_rad_ = 0.0;
  eccentricity_x_ = 0.0;
  eccentricity_y_ = 0.0;
  inclination_rad_ = 0.0;
  raan_rad_ = 0.0;
}

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements(const double semi_major_axis_m, const double eccentricity_x,
                                                                 const double eccentricity_y, const double inclination_rad, const double raan_rad,
                                                                 const double mean_arg_latitude_epoch_rad)
    : semi_major_axis_m_(semi_major_axis_m),
      eccentricity_x_(eccentricity_x),
      eccentricity_y_(eccentricity_y),
      inclination_rad_(inclination_rad),
      raan_rad_(raan_rad),
      mean_arg_latitude_epoch_rad_(mean_arg_latitude_epoch_rad) {}

QuasiNonsingularOrbitalElements::QuasiNonsingularOrbitalElements(const OrbitalElements oe) {
  double mean_anomaly_rad = 0.0;  // since the epoch is the perigee pass time

  semi_major_axis_m_ = oe.GetSemiMajor();
  mean_arg_latitude_epoch_rad_ = oe.GetArgPerigee() + mean_anomaly_rad;
  eccentricity_x_ = oe.GetEccentricity() * cos(oe.GetArgPerigee());
  eccentricity_y_ = oe.GetEccentricity() * sin(oe.GetArgPerigee());
  inclination_rad_ = oe.GetInclination();
  raan_rad_ = oe.GetRaan();
}

QuasiNonsingularOrbitalElements ::~QuasiNonsingularOrbitalElements() {}

QuasiNonsingularOrbitalElements operator-(const QuasiNonsingularOrbitalElements lhs, const QuasiNonsingularOrbitalElements rhs) {
  double semi_major_axis_m = lhs.GetSemiMajor_m() - rhs.GetSemiMajor_m();
  double eccentricity_x = lhs.GetEccentricityX() - rhs.GetEccentricityX();
  double eccentricity_y = lhs.GetEccentricityY() - rhs.GetEccentricityY();
  double inclination_rad = lhs.GetInclination_rad() - rhs.GetInclination_rad();
  double raan_rad = lhs.GetRaan_rad() - rhs.GetRaan_rad();
  double mean_arg_latitude_epoch_rad = lhs.GetMeanArgLatEpoch_rad() - lhs.GetMeanArgLatEpoch_rad();

  QuasiNonsingularOrbitalElements out(semi_major_axis_m, eccentricity_x, eccentricity_y, inclination_rad, raan_rad, mean_arg_latitude_epoch_rad);

  return out;
}
