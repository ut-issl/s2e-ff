#include <gtest/gtest.h>

#include "../src/Library/RelativeOrbit/QuasiNonsingularOrbitalElementDifferences.hpp"

TEST(QuasiNonsingularOrbitalElementDifferences, ConstructorWithOe) {
  // reference
  const double reference_semi_major_axis_m = 6896e3;
  const double reference_eccentricity_x = 0.0;  // Test singular point
  const double reference_eccentricity_y = 0.0;  // Test singular point
  const double reference_inclination_rad = 1.7;
  const double reference_raan_rad = 5.93;
  const double reference_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements reference_qn_oe(reference_semi_major_axis_m, reference_eccentricity_x, reference_eccentricity_y,
                                                  reference_inclination_rad, reference_raan_rad, reference_true_latitude_angle_rad);
  // target
  const double target_semi_major_axis_m = 6896e3;
  const double target_eccentricity_x = 0.05;
  const double target_eccentricity_y = 0.03;
  const double target_inclination_rad = 1.7;
  const double target_raan_rad = 5.93;
  const double target_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements target_qn_oe(target_semi_major_axis_m, target_eccentricity_x, target_eccentricity_y, target_inclination_rad,
                                               target_raan_rad, target_true_latitude_angle_rad);

  QuasiNonsingularOrbitalElementDifferences qn_oe(reference_qn_oe, target_qn_oe);
  // OEs
  EXPECT_NEAR(target_semi_major_axis_m - reference_semi_major_axis_m, qn_oe.GetDiffQuasiNonSingularOrbitalElements().GetSemiMajor_m(), 1);
  EXPECT_NEAR(target_eccentricity_x - reference_eccentricity_x, qn_oe.GetDiffQuasiNonSingularOrbitalElements().GetEccentricityX(), 1e-6);
  EXPECT_NEAR(target_eccentricity_y - reference_eccentricity_y, qn_oe.GetDiffQuasiNonSingularOrbitalElements().GetEccentricityY(), 1e-6);
  EXPECT_NEAR(target_inclination_rad - reference_inclination_rad, qn_oe.GetDiffQuasiNonSingularOrbitalElements().GetInclination_rad(), 1e-3);
  EXPECT_NEAR(target_raan_rad - reference_raan_rad, qn_oe.GetDiffQuasiNonSingularOrbitalElements().GetRaan_rad(), 1e-3);
  EXPECT_NEAR(target_true_latitude_angle_rad - reference_true_latitude_angle_rad, qn_oe.GetDiffQuasiNonSingularOrbitalElements().GetTrueLatAng_rad(),
              1e-3);
}

TEST(QuasiNonsingularOrbitalElementDifferences, ConstructorWithPositionVelocity) {
  const double mu_m3_s2 = 3.986008e14;
  // reference satellite
  const double reference_semi_major_axis_m = 6896e3;
  const double reference_eccentricity_x = 0.002;
  const double reference_eccentricity_y = 0.004;
  const double reference_inclination_rad = 1.7;
  const double reference_raan_rad = 5.93;
  const double reference_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements reference_qn_oe(reference_semi_major_axis_m, reference_eccentricity_x, reference_eccentricity_y,
                                                  reference_inclination_rad, reference_raan_rad, reference_true_latitude_angle_rad);
  // target relative position and velocity
  libra::Vector<3> position_rtn_m;
  position_rtn_m[0] = -0.000213852;
  position_rtn_m[1] = -11.4122;
  position_rtn_m[2] = 4.65733;
  libra::Vector<3> velocity_rtn_m_s;
  velocity_rtn_m_s[0] = 1.1448E-06;
  velocity_rtn_m_s[1] = 0.0;
  velocity_rtn_m_s[2] = 0.005298;

  QuasiNonsingularOrbitalElementDifferences qn_oe(reference_qn_oe, position_rtn_m, velocity_rtn_m_s, mu_m3_s2);
  libra::Vector<3> calc_position_rtn_m = qn_oe.CalcRelativePositionCircularApprox_rtn_m();
  libra::Vector<3> calc_velocity_rtn_m_s = qn_oe.CalcRelativeVelocityCircularApprox_rtn_m_s(mu_m3_s2);

  // Compare position and velocity
  EXPECT_NEAR(calc_position_rtn_m[0], position_rtn_m[0], 1e-5);
  EXPECT_NEAR(calc_position_rtn_m[1], position_rtn_m[1], 1e-5);
  EXPECT_NEAR(calc_position_rtn_m[2], position_rtn_m[2], 1e-5);
  EXPECT_NEAR(calc_velocity_rtn_m_s[0], velocity_rtn_m_s[0], 1e-5);
  EXPECT_NEAR(calc_velocity_rtn_m_s[1], velocity_rtn_m_s[1], 1e-5);
  EXPECT_NEAR(calc_velocity_rtn_m_s[2], velocity_rtn_m_s[2], 1e-5);
}
