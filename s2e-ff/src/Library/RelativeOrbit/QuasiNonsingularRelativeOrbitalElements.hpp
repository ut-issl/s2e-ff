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

  // Calculation
  libra::Vector<3> CalcRelativePositionCircularApprox_rtn_m(const double arg_lat_rad);  //!< Calculate relative position from ROE when near circular chief orbit

  // Getter
  inline double GetDeltaSemiMajor() const { return d_semi_major_axis_; }
  inline double GetDeltaMeanLongitude() const { return d_mean_longitude_; }
  inline double GetDeltaEccentricityX() const { return d_eccentricity_x_; }
  inline double GetDeltaEccentricityY() const { return d_eccentricity_y_; }
  inline double GetDeltaInclinationX() const { return d_inclination_x_; }
  inline double GetDeltaInclinationY() const { return d_inclination_y_; }

 private:
  // Reference orbit information
  double semi_major_axis_ref_m_;  //!< Semi major axis of reference orbit [m]

  // Relative Orbital Elements
  double d_semi_major_axis_; //!< Relative semi major axis [-]
  double d_mean_longitude_;  //!< Relative mean longitude [-]
  double d_eccentricity_x_;  //!< Relative eccentricity vector X component [-]
  double d_eccentricity_y_;  //!< Relative eccentricity vector Y component [-]
  double d_inclination_x_;   //!< Relative inclination vector X component [-]
  double d_inclination_y_;   //!< Relative inclination vector Y component [-]
};

#endif
