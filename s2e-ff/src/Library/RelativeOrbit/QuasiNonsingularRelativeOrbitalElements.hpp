#ifndef QUASI_NONSINGULAR_RELATIVE_ORBITAL_ELEMENTS_H_
#define QUASI_NONSINGULAR_RELATIVE_ORBITAL_ELEMENTS_H_

#include <Library/math/Vector.hpp>

#include "../Orbit/QuasiNonsingularOrbitalElements.hpp"

/**
 * @class QuasiNonsingularRelativeOrbitalElements
 * @brief Relative orbital elements defined by eccentricity/inclination vectors to avoid singularity when the eccentricity is near zero.
 * @note  Relative orbital elements(ROEs) is defined as a set of six unique linear or nonlinear combinations of two orbital elements
 */
class QuasiNonsingularRelativeOrbitalElements {
 public:
  QuasiNonsingularRelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target);
  ~QuasiNonsingularRelativeOrbitalElements();
  libra::Vector<3> CalcRelativePositionRtn_m(const double arg_lat_rad);  //!< Calculate relative position from ROE

 private:
  // Reference orbit information
  double a_ref_m_;  //!< Semi major axis of reference orbit [m]

  // Relative Orbital Elements
  double delta_a_;       //!< Relative semi major axis [-]
  double delta_lambda_;  //!< Relative mean longitude [-]
  double delta_e_x_;     //!< Relative eccentricity vector X component [-]
  double delta_e_y_;     //!< Relative eccentricity vector Y component [-]
  double delta_i_x_;     //!< Relative inclination vector X component [-]
  double delta_i_y_;     //!< Relative inclination vector Y component [-]
};

#endif
