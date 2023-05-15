#include <gtest/gtest.h>

#include "../src/library/relative_orbit/quasi_nonsingular_relative_orbital_elements.hpp"

TEST(QuasiNonsingularRelativeOrbitalElements, ConstructorWithOe) {
  // reference
  const double reference_semi_major_axis_m = 6896e3;
  const double reference_eccentricity_x = 0.0;  // Test singular point
  const double reference_eccentricity_y = 0.0;  // Test singular point
  const double reference_inclination_rad = 1.7;
  const double reference_raan_rad = 5.93;
  const double reference_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements reference_qn_oe(reference_semi_major_axis_m, reference_true_latitude_angle_rad, reference_inclination_rad,
                                                  reference_eccentricity_x, reference_eccentricity_y, reference_raan_rad);
  // target
  const double target_semi_major_axis_m = 6896e3;
  const double target_eccentricity_x = 0.05;
  const double target_eccentricity_y = 0.03;
  const double target_inclination_rad = 1.7;
  const double target_raan_rad = 5.93;
  const double target_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements target_qn_oe(target_semi_major_axis_m, target_true_latitude_angle_rad, target_inclination_rad,
                                               target_eccentricity_x, target_eccentricity_y, target_raan_rad);

  QuasiNonsingularRelativeOrbitalElements qn_roe(reference_qn_oe, target_qn_oe);

  EXPECT_NEAR((target_semi_major_axis_m - reference_semi_major_axis_m) / reference_semi_major_axis_m, qn_roe.GetDeltaSemiMajor(), 1);
  // When reference is circular orbit
  EXPECT_NEAR(target_true_latitude_angle_rad - reference_true_latitude_angle_rad, qn_roe.GetDeltaMeanLongitude(), 1e-6);

  EXPECT_NEAR(target_eccentricity_x - reference_eccentricity_x, qn_roe.GetDeltaEccentricityX(), 1e-6);
  EXPECT_NEAR(target_eccentricity_y - reference_eccentricity_y, qn_roe.GetDeltaEccentricityY(), 1e-6);

  EXPECT_NEAR(target_inclination_rad - reference_inclination_rad, qn_roe.GetDeltaInclinationX(), 1e-3);
  EXPECT_NEAR((target_raan_rad - reference_raan_rad) * sin(reference_inclination_rad), qn_roe.GetDeltaInclinationY(), 1e-3);
}

TEST(QuasiNonsingularRelativeOrbitalElements, ConstructorWithPositionVelocity) {
  const double mu_m3_s2 = 3.986008e14;
  // reference satellite
  const double reference_semi_major_axis_m = 6896e3;

  // target relative position and velocity
  libra::Vector<3> position_rtn_m;
  position_rtn_m[0] = -0.000213852;
  position_rtn_m[1] = -11.4122;
  position_rtn_m[2] = 4.65733;
  libra::Vector<3> velocity_rtn_m_s;
  velocity_rtn_m_s[0] = 1.1448E-06;
  velocity_rtn_m_s[1] = 0.0;
  velocity_rtn_m_s[2] = 0.005298;

  QuasiNonsingularRelativeOrbitalElements qn_roe(reference_semi_major_axis_m, position_rtn_m, velocity_rtn_m_s, mu_m3_s2);

  double u = 0;
  libra::Vector<3> calc_position_rtn_m = qn_roe.CalcRelativePositionCircularApprox_rtn_m(u);
  libra::Vector<3> calc_velocity_rtn_m_s = qn_roe.CalcRelativeVelocityCircularApprox_rtn_m_s(u, mu_m3_s2);

  // Compare position and velocity
  EXPECT_NEAR(calc_position_rtn_m[0], position_rtn_m[0], 1e-5);
  EXPECT_NEAR(calc_position_rtn_m[1], position_rtn_m[1], 1e-5);
  EXPECT_NEAR(calc_position_rtn_m[2], position_rtn_m[2], 1e-5);
  EXPECT_NEAR(calc_velocity_rtn_m_s[0], velocity_rtn_m_s[0], 1e-5);
  EXPECT_NEAR(calc_velocity_rtn_m_s[1], velocity_rtn_m_s[1], 1e-5);
  EXPECT_NEAR(calc_velocity_rtn_m_s[2], velocity_rtn_m_s[2], 1e-5);
}
