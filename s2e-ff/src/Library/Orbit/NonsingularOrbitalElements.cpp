#include "NonsingularOrbitalElements.hpp"

NonsingularOrbitalElements::NonsingularOrbitalElements(const OrbitalElements oe) {
  double mean_anomaly_rad = 0.0;  // since the epoch is the perigee pass time

  semi_major_axis_m_ = oe.GetSemiMajor();
  mean_arg_latitude_epoch_rad_ = oe.GetArgPerigee() + mean_anomaly_rad;
  eccentricity_x_ = oe.GetEccentricity() * cos(oe.GetArgPerigee());
  eccentricity_y_ = oe.GetEccentricity() * sin(oe.GetArgPerigee());
  inclination_rad_ = oe.GetInclination();
  raan_rad_ = oe.GetRaan();
}
