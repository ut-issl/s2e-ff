/**
 * @file QuasiNonsingularOrbitalElementDifferences.hpp
 * @brief Orbital elements differences to avoid singularity when the eccentricity is near zero.
 */

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
  /**
   * @fn QuasiNonsingularOrbitalElementDifferences
   * @brief Constructor initialized with tow quasi-nonsingular orbital elements
   * @param qns_oe_reference: Quasi-nonsingular orbital elements of the reference spacecraft
   * @param qns_oe_target: Quasi-nonsingular orbital elements of the target spacecraft
   */
  QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                            const QuasiNonsingularOrbitalElements qns_oe_target);
  /**
   * @fn ~QuasiNonsingularOrbitalElementDifferences
   * @brief deconstructor
   */
  ~QuasiNonsingularOrbitalElementDifferences();

  // Calculation
  /**
   * @fn CalcRelativePositionCircularApprox_rtn_m
   * @brief Calculate the relative position of target spacecraft with circular approximation
   * @params true_anomaly_rad: True anomaly [rad]
   * @return Relative position vector in RTN frame of reference spacecraft
   */
  libra::Vector<3> CalcRelativePositionCircularApprox_rtn_m(const double true_anomaly_rad);

  // Getter
  /**
   * @fn GetDiffSemiMajor_m
   * @brief Return difference of semi-major axis [m]
   */
  inline double GetDiffSemiMajor_m() const { return d_semi_major_axis_m_; }
  /**
   * @fn GetDiffEccentricityX
   * @brief Return difference of eccentricity vector X component
   */
  inline double GetDiffEccentricityX() const { return d_eccentricity_x_; }
  /**
   * @fn GetDiffEccentricityY
   * @brief Return difference of eccentricity vector Y component
   */
  inline double GetDiffEccentricityY() const { return d_eccentricity_y_; }
  /**
   * @fn GetDiffInclination_rad
   * @brief Return difference of inclination [rad]
   */
  inline double GetDiffInclination_rad() const { return d_inclination_rad_; }
  /**
   * @fn GetDiffRaan_rad
   * @brief Return difference of RAAN [rad]
   */
  inline double GetDiffRaan_rad() const { return d_raan_rad_; }
  /**
   * @fn GetDiffMeanArgLatEpoch_rad
   * @brief Return difference of argument of latitude [rad]
   */
  inline double GetDiffMeanArgLatEpoch_rad() const { return d_mean_arg_latitude_epoch_rad_; }

 private:
  // Reference orbit information
  double semi_major_axis_ref_m_;  //!< Semi major axis of reference orbit [m]

  // Orbital Element Differences
  double d_semi_major_axis_m_;            //!< Difference of semi major axis of reference orbit [m]
  double d_eccentricity_x_;               //!< Difference of eccentricity vector X component
  double d_eccentricity_y_;               //!< Difference of eccentricity vector Y component
  double d_inclination_rad_;              //!< Difference of inclination [rad]
  double d_raan_rad_;                     //!< Difference of RAAN [rad]
  double d_mean_arg_latitude_epoch_rad_;  //!< Difference of argument of latitude [rad]
};

#endif
