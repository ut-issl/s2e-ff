/**
 * @file QuasiNonsingularOrbitalElements.hpp
 * @brief Orbital elements avoid singularity when the eccentricity is near zero.
 */

#ifndef QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_
#define QUASI_NONSINGULAR_ORBITAL_ELEMENTS_H_

#include <library/math/s2e_math.hpp>
#include <library/math/vector.hpp>

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
   * @param[in] semi_major_axis_m: Semi major axis [m]
   * @param[in] true_latitude_angle_rad: True latitude angle [rad]
   * @param[in] inclination_rad: Inclination [rad]
   * @param[in] eccentricity_x: Eccentricity X component
   * @param[in] eccentricity_y: Eccentricity Y component
   * @param[in] raan_rad: Right Ascension of the Ascending Node [rad]
   */
  QuasiNonsingularOrbitalElements(const double semi_major_axis_m, const double true_latitude_angle_rad, const double inclination_rad,
                                  const double eccentricity_x, const double eccentricity_y, const double raan_rad);
  /**
   * @fn QuasiNonsingularOrbitalElements
   * @brief Constructor initialized with position and velocity
   * @param[in] mu_m3_s2: Gravity constant [m3/s2]
   * @param[in] position_i_m: Position vector in the inertial frame [m]
   * @param[in] velocity_i_m_s: Velocity vector in the inertial frame [m/s]
   */
  QuasiNonsingularOrbitalElements(const double mu_m3_s2, const libra::Vector<3> position_i_m, const libra::Vector<3> velocity_i_m_s);
  /**
   * @fn QuasiNonsingularOrbitalElements
   * @brief Constructor initialized with orbital elements in Vector expression
   * @note Order is semi-major, true latitude, inclination, e_x, e_y, and RAAN
   */
  QuasiNonsingularOrbitalElements(const libra::Vector<6> oe_vector);
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
  /**
   * @fn GetAsVector
   * @brief Return Orbital elements as vector expression
   * @note Order is semi-major, true latitude, inclination, e_x, e_y, and RAAN
   */
  inline libra::Vector<6> GetAsVector() const {
    libra::Vector<6> oe;
    oe[0] = semi_major_axis_m_;
    oe[1] = true_latitude_angle_rad_;
    oe[2] = inclination_rad_;
    oe[3] = eccentricity_x_;
    oe[4] = eccentricity_y_;
    oe[5] = raan_rad_;
    return oe;
  }

  // Setter
  /**
   * @fn SetTrueLAtAng_rad
   * @brief Set True latitude angle [rad]
   */
  inline void GetTrueLatAng_rad(const double true_latitude_angle_rad) { true_latitude_angle_rad_ = libra::WrapTo2Pi(true_latitude_angle_rad); }

 private:
  double semi_major_axis_m_;        //!< Semi major axis [m]
  double true_latitude_angle_rad_;  //!< True latitude angle (argment of periapsis + true anomaly) [rad]
  double inclination_rad_;          //!< Inclination [rad]
  double eccentricity_x_;           //!< e * cos(arg_peri)
  double eccentricity_y_;           //!< e * sin(arg_peri)
  double raan_rad_;                 //!< Right Ascension of the Ascending Node [rad]

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
