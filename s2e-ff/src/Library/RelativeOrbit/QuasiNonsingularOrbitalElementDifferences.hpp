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
  // libra::Vector<3> CalcRelativePositionRtn_m(const double arg_lat_rad);  //!< Calculate relative position from ROE

 private:
  // Relative Orbital Elements
  double d_semi_major_axis_m_;
  double d_eccentricity_x_;
  double d_eccentricity_y_;
  double d_inclination_rad_;
  double d_raan_rad_;
  double d_mean_arg_latitude_epoch_rad_;
};

#endif
