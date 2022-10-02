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

  // Getter
  inline double GetSemiMajorRef_m() const { return semi_major_axis_ref_m_; }
  inline double GetDeltaSemiMajor() const { return delta_semi_major_axis_; }
  inline double GetDeltaMeanLongitude() const { return delta_mean_longitude_; }
  inline double GetDeltaEccentricityX() const { return delta_eccentricity_x_; }
  inline double GetDeltaEccentricityY() const { return delta_eccentricity_y_; }
  inline double GetDeltaInclinationX() const { return delta_inclination_x_; }
  inline double GetDeltaInclinationY() const { return delta_inclination_y_; }

 private:
  // Reference orbit information
  double semi_major_axis_ref_m_;  //!< Semi major axis of reference orbit [m]

  // Relative Orbital Elements
  double delta_semi_major_axis_; //!< Relative semi major axis [-]
  double delta_mean_longitude_;  //!< Relative mean longitude [-]
  double delta_eccentricity_x_;  //!< Relative eccentricity vector X component [-]
  double delta_eccentricity_y_;  //!< Relative eccentricity vector Y component [-]
  double delta_inclination_x_;   //!< Relative inclination vector X component [-]
  double delta_inclination_y_;   //!< Relative inclination vector Y component [-]
};

#endif
