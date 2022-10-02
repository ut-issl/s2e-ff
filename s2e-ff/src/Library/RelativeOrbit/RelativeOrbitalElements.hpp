#ifndef RELATIVE_ORBITAL_ELEMENTS_H_
#define RELATIVE_ORBITAL_ELEMENTS_H_

#include <Library/math/Vector.hpp>

#include "../Orbit/QuasiNonsingularOrbitalElements.hpp"

class RelativeOrbitalElements {
 public:
  RelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target);
  ~RelativeOrbitalElements();
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
