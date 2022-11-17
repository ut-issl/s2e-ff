#ifndef QUASI_NONSINGULAR_ORBITAL_ELEMENT_DIFFERENCES_H_
#define QUASI_NONSINGULAR_ORBITAL_ELEMENT_DIFFERENCES_H_

#include <Library/math/Vector.hpp>

#include "../Orbit/QuasiNonsingularOrbitalElements.hpp"

/**
 * @class QuasiNonsingularOrbitalElementDifferences
 * @brief Orbital element differences defined by eccentricity/inclination vectors to avoid singularity when the eccentricity is near zero.
 * @note  Orbital element differences(OEDs) is defined as the arithmetic difference between two orbital elements
 */
class QuasiNonsingularOrbitalElementDifferences {
 public:
  QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target);
  ~QuasiNonsingularOrbitalElementDifferences();
  
  // Calculation
  libra::Vector<3> CalcRelativePositionCircularApprox_rtn_m(const double true_anomaly_rad);  //!< Calculate relative position from OED when near circular chief orbit

  // Getter
  inline double GetDiffSemiMajor_m() const { return d_semi_major_axis_m_; }
  inline double GetDiffEccentricityX() const { return d_eccentricity_x_; }
  inline double GetDiffEccentricityY() const { return d_eccentricity_y_; }
  inline double GetDiffInclination_rad() const { return d_inclination_rad_; }
  inline double GetDiffRaan_rad() const { return d_raan_rad_; }
  inline double GetDiffMeanArgLatEpoch_rad() const { return d_mean_arg_latitude_epoch_rad_; }

 private:
  // Reference orbit information
  double semi_major_axis_ref_m_;  //!< Semi major axis of reference orbit [m]

  // Orbital Element Differences
  double d_semi_major_axis_m_;
  double d_eccentricity_x_;
  double d_eccentricity_y_;
  double d_inclination_rad_;
  double d_raan_rad_;
  double d_mean_arg_latitude_epoch_rad_;
};

#endif
