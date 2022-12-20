/**
 * @file QuasiNonsingularOrbitalElements.hpp
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */

#ifndef QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_
#define QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_

#include <Library/Orbit/OrbitalElements.h>

/**
 * @class QuasiNonsingularOrbitalElements
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */
class QuasiNonsingularOrbitalElements {
 public:
  /**
   * @fn QuasiNonsingularOrbitalElements
   * @brief Default Constructor
   */
  QuasiNonsingularOrbitalElements();
  /**
   * @fn QuasiNonsingularOrbitalElements
   * @brief Constructor initialized with values
   */
  QuasiNonsingularOrbitalElements(const double semi_major_axis_m, const double eccentricity_x, const double eccentricity_y,
                                  const double inclination_rad, const double raan_rad, const double true_latitude_angle_rad);
  /**
   * @fn ~QuasiNonsingularOrbitalElements
   * @brief Destructor
   */
  ~QuasiNonsingularOrbitalElements();

  // Getter
  /**
   * @fn GetSemiMajor_m
   * @brief Return semi-major axis [m]
   */
  inline double GetSemiMajor_m() const { return semi_major_axis_m_; }
  /**
   * @fn GetEccentricityX
   * @brief Return X component of eccentricity vector
   */
  inline double GetEccentricityX() const { return eccentricity_x_; }
  /**
   * @fn GetEccentricityY
   * @brief Return Y component of eccentricity vector
   */
  inline double GetEccentricityY() const { return eccentricity_y_; }
  /**
   * @fn GetInclination_rad
   * @brief Return inclination [rad]
   */
  inline double GetInclination_rad() const { return inclination_rad_; }
  /**
   * @fn GetRaan_rad
   * @brief Return Right Ascension of the Ascending Node [rad]
   */
  inline double GetRaan_rad() const { return raan_rad_; }
  /**
   * @fn GetTrueLatAng_rad
   * @brief Return True latitude angle [rad]
   */
  inline double GetTrueLatAng_rad() const { return true_latitude_angle_rad_; }
  /**
   * @fn GetSemiLatusRectum_m
   * @brief Return Semi-latus rectum [m]
   */
  inline double GetSemiLatusRectum_m() const { return semi_latus_rectum_m_; }
  /**
   * @fn GetRadius_m
   * @brief Return Current radius [m]
   */
  inline double GetRadius_m() const { return radius_m_; }

 private:
  double semi_major_axis_m_;        //!< Semi major axis [m]
  double eccentricity_x_;           //!< e * cos(arg_peri)
  double eccentricity_y_;           //!< e * sin(arg_peri)
  double inclination_rad_;          //!< Inclination [rad]
  double raan_rad_;                 //!< Right Ascension of the Ascending Node [rad]
  double true_latitude_angle_rad_;  //!< True latitude angle (arg_peri + true anomaly) [rad]

  // Orbit states
  double semi_latus_rectum_m_;  //!< Semi-latus rectum [m]
  double radius_m_;             //!< Current radius [m]

  /**
   * @fn CalcOrbitParameters
   * @brief Calculate parameters for orbit
   */
  void CalcOrbitParameters();
};

/**
 * @fn Operator -
 * @brief Calculate subtract of two quasi-nonsingular orbital elements
 */
QuasiNonsingularOrbitalElements operator-(const QuasiNonsingularOrbitalElements lhs, const QuasiNonsingularOrbitalElements rhs);

#endif
