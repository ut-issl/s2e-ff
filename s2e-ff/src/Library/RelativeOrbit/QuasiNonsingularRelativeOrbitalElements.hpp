/**
 * @file QuasiNonsingularRelativeOrbitalElements.hpp
 * @brief Relative orbital elements defined by eccentricity/inclination vectors to avoid singularity when the eccentricity is near zero.
 */

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
  /**
   * @fn QuasiNonsingularRelativeOrbitalElements
   * @brief Constructor initialized with tow quasi-nonsingular orbital elements
   * @param [in] qns_oe_reference: Quasi-nonsingular orbital elements of the reference spacecraft
   * @param [in] qns_oe_target: Quasi-nonsingular orbital elements of the target spacecraft
   */
  QuasiNonsingularRelativeOrbitalElements(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                          const QuasiNonsingularOrbitalElements qns_oe_target);
  /**
   * @fn QuasiNonsingularRelativeOrbitalElements
   * @brief Constructor initialized with relative position and velocity
   * @param [in] semi_major_axis_ref_m: Semi-major axis of the reference satellite orbit [m]
   * @param [in] relative_position_rtn_m: Relative position of target satellite in the reference satellite's RTN frame [m]
   * @param [in] relative_velocity_rtn_m_s: Relative velocity of target satellite in the reference satellite's RTN frame [m/s]
   * @param [in] mu_m3_s2: Gravity constant of the center body [m3/s2]
   */
  QuasiNonsingularRelativeOrbitalElements(const double semi_major_axis_ref_m, const libra::Vector<3> relative_position_rtn_m,
                                          const libra::Vector<3> relative_velocity_rtn_m_s, const double mu_m3_s2);
  /**
   * @fn ~QuasiNonsingularRelativeOrbitalElements
   * @brief Destructor
   */
  ~QuasiNonsingularRelativeOrbitalElements();

  // Calculation
  /**
   * @fn CalcRelativePositionCircularApprox_rtn_m
   * @brief Calculate the relative position of target spacecraft with circular approximation
   * @param [in] mean_arg_lat_rad: Mean argument of latitude [rad]
   * @return Relative position vector in RTN frame of reference spacecraft [m]
   */
  libra::Vector<3> CalcRelativePositionCircularApprox_rtn_m(const double mean_arg_lat_rad);
  /**
   * @fn CalcRelativeVelocityCircularApprox_rtn_m_s
   * @brief Calculate the relative position of target spacecraft with circular approximation
   * @param [in] mean_arg_lat_rad: Mean argument of latitude [rad]
   * @param [in] mu_m3_s2: Gravity constant of the center body [m3/s2]
   * @return Relative velocity vector in RTN frame of reference spacecraft [m/s]
   */
  libra::Vector<3> CalcRelativeVelocityCircularApprox_rtn_m_s(const double mean_arg_lat_rad, const double mu_m3_s2);

  // Getter
  /**
   * @fn GetDeltaSemiMajor
   * @brief Return Relative semi major axis [-]
   */
  inline double GetDeltaSemiMajor() const { return d_semi_major_axis_; }
  /**
   * @fn GetDeltaSemiMajor
   * @brief Return Relative mean longitude [-]
   */
  inline double GetDeltaMeanLongitude() const { return d_mean_longitude_; }
  /**
   * @fn GetDeltaSemiMajor
   * @brief Return Relative eccentricity vector X component [-]
   */
  inline double GetDeltaEccentricityX() const { return d_eccentricity_x_; }
  /**
   * @fn GetDeltaSemiMajor
   * @brief Return Relative eccentricity vector Y component [-]
   */
  inline double GetDeltaEccentricityY() const { return d_eccentricity_y_; }
  /**
   * @fn GetDeltaSemiMajor
   * @brief Return Relative inclination vector X component [-]
   */
  inline double GetDeltaInclinationX() const { return d_inclination_x_; }
  /**
   * @fn GetDeltaSemiMajor
   * @brief Return Relative inclination vector Y component [-]
   */
  inline double GetDeltaInclinationY() const { return d_inclination_y_; }

 private:
  // Reference orbit information
  double semi_major_axis_ref_m_;  //!< Semi major axis of reference orbit [m]

  // Relative Orbital Elements
  double d_semi_major_axis_;  //!< Relative semi major axis [-]
  double d_mean_longitude_;   //!< Relative mean longitude [-]
  double d_eccentricity_x_;   //!< Relative eccentricity vector X component [-]
  double d_eccentricity_y_;   //!< Relative eccentricity vector Y component [-]
  double d_inclination_x_;    //!< Relative inclination vector X component [-]
  double d_inclination_y_;    //!< Relative inclination vector Y component [-]

  /**
   * @fn CalcDiffMeanArgLat_rad
   * @brief Calculate difference of mean argument of latitude [rad]
   * @note Mean argument of latitude = argument of periapsis + mean anomaly
   * @param [in] qns_oe_reference: Quasi-nonsingular orbital elements of the reference spacecraft
   * @param [in] qns_oe_target: Quasi-nonsingular orbital elements of the target spacecraft
   * @return Difference of mean argument of latitude [rad]
   */
  double CalcDiffMeanArgLat_rad(const QuasiNonsingularOrbitalElements qns_oe_reference, const QuasiNonsingularOrbitalElements qns_oe_target);
};

#endif
