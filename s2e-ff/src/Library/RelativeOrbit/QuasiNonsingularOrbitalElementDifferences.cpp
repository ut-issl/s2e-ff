/**
 * @file QuasiNonsingularOrbitalElementDifferences.cpp
 * @brief Orbital elements differences to avoid singularity when the eccentricity is near zero.
 */

#include "QuasiNonsingularOrbitalElementDifferences.hpp"

#include <Library/math/MatVec.hpp>
#include <cmath>

QuasiNonsingularOrbitalElementDifferences::QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                                                                     const QuasiNonsingularOrbitalElements qns_oe_target)
    : qns_oe_reference_(qns_oe_reference) {
  diff_qns_oe_ = qns_oe_target - qns_oe_reference;
}

QuasiNonsingularOrbitalElementDifferences::QuasiNonsingularOrbitalElementDifferences(const QuasiNonsingularOrbitalElements qns_oe_reference,
                                                                                     const libra::Vector<3> relative_position_rtn_m,
                                                                                     const libra::Vector<3> relative_velocity_rtn_m_s,
                                                                                     const double mu_m3_s2)
    : qns_oe_reference_(qns_oe_reference) {
  // Reference orbit variables
  const double a = qns_oe_reference_.GetSemiMajor_m();
  const double ex = qns_oe_reference_.GetEccentricityX();
  const double ey = qns_oe_reference_.GetEccentricityY();
  const double i = qns_oe_reference_.GetInclination_rad();
  const double sin_i = sin(i);
  const double cot_i = cos(i) / sin_i;
  const double theta = qns_oe_reference_.GetTrueLatAng_rad();
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double cos_2theta = cos(2.0 * theta);
  const double sin_2theta = sin(2.0 * theta);
  const double p = qns_oe_reference_.GetSemiLatusRectum_m();
  const double h = sqrt(mu_m3_s2 * p);  //!< Orbit angular momentum
  const double r = qns_oe_reference_.GetRadius_m();

  // Calculation
  const double v_r = h / p * (ex * sin_theta - ey * cos_theta);
  const double v_t = h / p * (1.0 + ex * cos_theta + ey * sin_theta);

  const double alpha = a / r;
  const double nu = v_r / v_t;
  const double rho = r / p;
  const double kappa_1 = alpha * (1.0 / rho - 1.0);
  const double kappa_2 = alpha * nu * nu / rho;

  libra::Matrix<6, 6> conversion_to_oed(0.0);
  // For semi-major axis
  conversion_to_oed[0][0] = 2.0 * alpha * (2.0 + 3.0 * kappa_1 + 2.0 * kappa_2);
  conversion_to_oed[0][1] = -2.0 * alpha * nu * (1.0 + 2.0 * kappa_1 + kappa_2);
  conversion_to_oed[0][3] = 2.0 * alpha * alpha * nu * p / v_t;
  conversion_to_oed[0][4] = 2.0 * alpha * (1.0 + 2.0 * kappa_1 + kappa_2) / v_t;
  // For true latitude angle
  conversion_to_oed[1][1] = 1.0 / r;
  conversion_to_oed[1][2] = cot_i / r * (cos_theta + nu * sin_theta);
  conversion_to_oed[1][5] = -1.0 * sin_theta * cot_i / v_t;
  // for inclination
  conversion_to_oed[2][2] = (sin_theta - nu * cos_theta) / r;
  conversion_to_oed[2][5] = cos_theta / v_t;
  // For eccentricity vector X
  conversion_to_oed[3][0] = (3.0 * cos_theta + 2.0 * nu * sin_theta) / (rho * r);
  conversion_to_oed[3][1] = -1.0 * (nu * nu * sin_theta / rho + ex * sin_2theta - ey * cos_2theta) / r;
  conversion_to_oed[3][2] = -1.0 * ey * cot_i * (cos_theta + nu * sin_theta) / r;
  conversion_to_oed[3][3] = sin_theta / (rho * v_t);
  conversion_to_oed[3][4] = (2.0 * cos_theta + nu * sin_theta) / (rho * v_t);
  conversion_to_oed[3][5] = ey * cot_i * sin_theta / v_t;
  // For eccentricity vector Y
  conversion_to_oed[4][0] = (3.0 * sin_theta - 2.0 * nu * cos_theta) / (rho * r);
  conversion_to_oed[4][1] = (nu * nu * cos_theta / rho + ey * sin_2theta + ex * cos_2theta) / r;
  conversion_to_oed[4][2] = ex * cot_i * (cos_theta + nu * sin_theta) / r;
  conversion_to_oed[4][3] = -1.0 * cos_theta / (rho * v_t);
  conversion_to_oed[4][4] = (2.0 * sin_theta - nu * cos_theta) / (rho * v_t);
  conversion_to_oed[4][5] = -1.0 * ex * cot_i * sin_theta / v_t;
  // For RAAN
  conversion_to_oed[5][2] = -1.0 * (cos_theta + nu * sin_theta) / (r * sin_i);
  conversion_to_oed[5][5] = sin_theta / (v_t * sin_i);

  // Output
  libra::Vector<6> position_and_velocity;
  for (size_t i = 0; i < 3; i++) {
    position_and_velocity[i] = relative_position_rtn_m[i];
    position_and_velocity[i + 3] = relative_velocity_rtn_m_s[i];
  }
  libra::Vector<6> relative_oed;
  relative_oed = conversion_to_oed * position_and_velocity;
  QuasiNonsingularOrbitalElements relative_oed_tmp(relative_oed[0], relative_oed[3], relative_oed[4], relative_oed[2], relative_oed[5],
                                                   relative_oed[1]);
  diff_qns_oe_ = relative_oed_tmp;
}

QuasiNonsingularOrbitalElementDifferences::~QuasiNonsingularOrbitalElementDifferences() {}

libra::Vector<3> QuasiNonsingularOrbitalElementDifferences::CalcRelativePositionCircularApprox_rtn_m() {
  // Reference orbit variables
  const double a = qns_oe_reference_.GetSemiMajor_m();
  const double ex = qns_oe_reference_.GetEccentricityX();
  const double ey = qns_oe_reference_.GetEccentricityY();
  const double i = qns_oe_reference_.GetInclination_rad();
  const double theta = qns_oe_reference_.GetTrueLatAng_rad();
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double p = qns_oe_reference_.GetSemiLatusRectum_m();
  const double r = qns_oe_reference_.GetRadius_m();

  // Relative orbit variables
  const double d_a = diff_qns_oe_.GetSemiMajor_m();
  const double d_ex = diff_qns_oe_.GetEccentricityX();
  const double d_ey = diff_qns_oe_.GetEccentricityY();
  const double d_theta = diff_qns_oe_.GetTrueLatAng_rad();
  const double d_i = diff_qns_oe_.GetInclination_rad();
  const double d_raan = diff_qns_oe_.GetRaan_rad();

  // Calculation
  const double v_r = (ex * sin_theta - ey * cos_theta);  // without h/p since it will be cancelled in v_r / v_t
  const double v_t = (1.0 + ex * cos_theta + ey * sin_theta);
  const double d_r = r / a * d_a + v_r / v_t * r * d_theta - r / p * ((2.0 * a * ex + r * cos_theta) * d_ex + (2.0 * a * ey + r * sin_theta) * d_ey);

  // Output
  libra::Vector<3> relative_position_rtn_m;
  relative_position_rtn_m[0] = d_r;
  relative_position_rtn_m[1] = r * (d_theta + d_raan * cos(i));
  relative_position_rtn_m[2] = r * (d_i * sin_theta - d_raan * cos_theta * sin(i));

  return relative_position_rtn_m;
}

libra::Vector<3> QuasiNonsingularOrbitalElementDifferences::CalcRelativeVelocityCircularApprox_rtn_m_s(const double mu_m3_s2) {
  // Reference orbit variables
  const double a = qns_oe_reference_.GetSemiMajor_m();
  const double ex = qns_oe_reference_.GetEccentricityX();
  const double ey = qns_oe_reference_.GetEccentricityY();
  const double i = qns_oe_reference_.GetInclination_rad();
  const double theta = qns_oe_reference_.GetTrueLatAng_rad();
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double p = qns_oe_reference_.GetSemiLatusRectum_m();
  const double h = sqrt(mu_m3_s2 * p);  //!< Orbit angular momentum
  const double r = qns_oe_reference_.GetRadius_m();

  // Relative orbit variables
  const double d_a = diff_qns_oe_.GetSemiMajor_m();
  const double d_ex = diff_qns_oe_.GetEccentricityX();
  const double d_ey = diff_qns_oe_.GetEccentricityY();
  const double d_theta = diff_qns_oe_.GetTrueLatAng_rad();
  const double d_i = diff_qns_oe_.GetInclination_rad();
  const double d_raan = diff_qns_oe_.GetRaan_rad();

  // Calculation
  const double v_r = h / p * (ex * sin_theta - ey * cos_theta);
  const double v_t = h / p * (1.0 + ex * cos_theta + ey * sin_theta);

  // Output
  libra::Vector<3> relative_velocity_rtn_m_s;
  relative_velocity_rtn_m_s[0] = -v_r / (2.0 * a) * d_a + (1.0 / r - 1.0 / p) * h * d_theta + (v_r * a * ex + h * sin_theta) * d_ex / p +
                                 (v_r * a * ey - h * cos_theta) * d_ey / p;
  relative_velocity_rtn_m_s[1] = -3.0 * v_t / (2.0 * a) * d_a - v_r * d_theta + (3.0 * v_t * a * ex + 2.0 * h * cos_theta) * d_ex / p +
                                 (3.0 * v_t * a * ey + 2.0 * h * sin_theta) * d_ey / p + v_r * cos(i) * d_raan;
  relative_velocity_rtn_m_s[2] = (v_t * cos_theta + v_r * sin_theta) * d_i + (v_t * sin_theta - v_r * cos_theta) * sin(i) * d_raan;

  return relative_velocity_rtn_m_s;
}
