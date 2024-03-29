/**
 * @file QuasiNonsingularOrbitalElementDifferences.hpp
 * @brief Orbital elements differences to avoid singularity when the eccentricity is near zero.
 */

#ifndef S2E_LIBRARY_RELATIVE_ORBIT_QUASI_NONSINGULAR_ORBITAL_ELEMENT_DIFFERENCES_HPP_
#define S2E_LIBRARY_RELATIVE_ORBIT_QUASI_NONSINGULAR_ORBITAL_ELEMENT_DIFFERENCES_HPP_

#include <library/math/vector.hpp>

#include "../orbit/quasi_nonsingular_orbital_elements.hpp"

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
   * @param [in] qns_oe_reference: Quasi-nonsingular orbital elements of the reference spacecraft
   * @param [in] qns_oe_target: Quasi-nonsingular orbital elements of the target spacecraft
   */
  QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                            const QuasiNonsingularOrbitalElements qns_oe_target);
  /**
   * @fn QuasiNonsingularOrbitalElementDifferences
   * @brief Constructor initialized with tow quasi-nonsingular orbital elements
   * @param [in] qns_oe_reference: Quasi-nonsingular orbital elements of the reference spacecraft
   * @param [in] relative_position_rtn_m: Relative position of target satellite in the reference satellite's RTN frame [m]
   * @param [in] relative_velocity_rtn_m_s: Relative velocity of target satellite in the reference satellite's RTN frame [m/s]
   * @param [in] mu_m3_s2: Gravity constant of the center body [m3/s2]
   */
  QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference, const libra::Vector<3> relative_position_rtn_m,
                                            const libra::Vector<3> relative_velocity_rtn_m_s, const double mu_m3_s2);
  /**
   * @fn ~QuasiNonsingularOrbitalElementDifferences
   * @brief Destructor
   */
  ~QuasiNonsingularOrbitalElementDifferences();

  // Calculation
  /**
   * @fn CalcRelativePosition_rtn_m
   * @brief Calculate the relative position of target spacecraft with short distance approximation
   * @return Relative position vector in RTN frame of reference spacecraft [m]
   */
  libra::Vector<3> CalcRelativePosition_rtn_m();
  /**
   * @fn CalcRelativeVelocity_rtn_m_s
   * @brief Calculate the relative velocity of target spacecraft with short distance approximation
   * @param [in] mu_m3_s2: Gravity constant of the center body [m3/s2]
   * @return Relative position vector in RTN frame of reference spacecraft [m/s]
   */
  libra::Vector<3> CalcRelativeVelocity_rtn_m_s(const double mu_m3_s2);

  // Getter
  /**
   * @fn GetDiffQuasiNonSingularOrbitalElements
   * @brief Return difference of quasi-nonsingular orbital elements
   */
  inline QuasiNonsingularOrbitalElements GetDiffQuasiNonSingularOrbitalElements() const { return diff_qns_oe_; }

 private:
  QuasiNonsingularOrbitalElements qns_oe_reference_;  //!< Quasi-nonsingular orbital elements of reference spacecraft
  QuasiNonsingularOrbitalElements diff_qns_oe_;       //!< Difference of quasi-nonsingular orbital elements
};

#endif  // S2E_LIBRARY_RELATIVE_ORBIT_QUASI_NONSINGULAR_ORBITAL_ELEMENT_DIFFERENCES_HPP_
