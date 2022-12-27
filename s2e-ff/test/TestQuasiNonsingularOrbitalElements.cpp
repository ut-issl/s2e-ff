#include <gtest/gtest.h>

#include "../src/Library/Orbit/QuasiNonsingularOrbitalElements.hpp"

TEST(QuasiNonsingularOrbitalElements, DefaultConstructor) {
  QuasiNonsingularOrbitalElements qn_oe;

  // OEs
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetSemiMajor_m());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetEccentricityX());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetEccentricityY());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetInclination_rad());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetRaan_rad());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetTrueLatAng_rad());
  // Parameters
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetSemiLatusRectum_m());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetRadius_m());
}

TEST(QuasiNonsingularOrbitalElements, ConstructorWithSingularOe) {
  const double semi_major_axis_m = 6896e3;
  const double eccentricity_x = 0.0;  // Test singular point
  const double eccentricity_y = 0.0;  // Test singular point
  const double inclination_rad = 1.7;
  const double raan_rad = 5.93;
  const double true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements qn_oe(semi_major_axis_m, eccentricity_x, eccentricity_y, inclination_rad, raan_rad, true_latitude_angle_rad);

  // OEs
  EXPECT_DOUBLE_EQ(semi_major_axis_m, qn_oe.GetSemiMajor_m());
  EXPECT_DOUBLE_EQ(eccentricity_x, qn_oe.GetEccentricityX());
  EXPECT_DOUBLE_EQ(eccentricity_y, qn_oe.GetEccentricityY());
  EXPECT_DOUBLE_EQ(inclination_rad, qn_oe.GetInclination_rad());
  EXPECT_DOUBLE_EQ(raan_rad, qn_oe.GetRaan_rad());
  EXPECT_DOUBLE_EQ(true_latitude_angle_rad, qn_oe.GetTrueLatAng_rad());
  // Parameters
  EXPECT_DOUBLE_EQ(semi_major_axis_m, qn_oe.GetSemiLatusRectum_m());
  EXPECT_DOUBLE_EQ(semi_major_axis_m, qn_oe.GetRadius_m());
}

TEST(QuasiNonsingularOrbitalElements, ConstructorWithOe) {
  const double semi_major_axis_m = 6896e3;
  const double eccentricity_x = 0.05;
  const double eccentricity_y = 0.03;
  const double inclination_rad = 1.7;
  const double raan_rad = 5.93;
  const double true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements qn_oe(semi_major_axis_m, eccentricity_x, eccentricity_y, inclination_rad, raan_rad, true_latitude_angle_rad);

  // OEs
  EXPECT_DOUBLE_EQ(semi_major_axis_m, qn_oe.GetSemiMajor_m());
  EXPECT_DOUBLE_EQ(eccentricity_x, qn_oe.GetEccentricityX());
  EXPECT_DOUBLE_EQ(eccentricity_y, qn_oe.GetEccentricityY());
  EXPECT_DOUBLE_EQ(inclination_rad, qn_oe.GetInclination_rad());
  EXPECT_DOUBLE_EQ(raan_rad, qn_oe.GetRaan_rad());
  EXPECT_DOUBLE_EQ(true_latitude_angle_rad, qn_oe.GetTrueLatAng_rad());
  // Parameters
  EXPECT_NEAR(6872553.6, qn_oe.GetSemiLatusRectum_m(), 1e-1);
  EXPECT_NEAR(6494189.8, qn_oe.GetRadius_m(), 1e-1);
}

TEST(QuasiNonsingularOrbitalElements, ConstructorWithPositionVelocity) {
  const double mu_m3_s2 = 3.986008e14;
  libra::Vector<3> position_i_m;
  position_i_m[0] = -5659121.225;
  position_i_m[1] = 2467374.064;
  position_i_m[2] = -3072838.471;
  const double r_norm_m = norm(position_i_m);
  libra::Vector<3> velocity_i_m_s;
  velocity_i_m_s[0] = 3517.005128;
  velocity_i_m_s[1] = -323.2731889;
  velocity_i_m_s[2] = -6734.568005;
  QuasiNonsingularOrbitalElements qn_oe(mu_m3_s2, position_i_m, velocity_i_m_s);

  // OEs
  EXPECT_NEAR(6.8993234545e6, qn_oe.GetSemiMajor_m(), 1);
  EXPECT_NEAR(-3.6369e-4, qn_oe.GetEccentricityX(), 1e-6);
  EXPECT_NEAR(-3.2297e-4, qn_oe.GetEccentricityY(), 1e-6);
  EXPECT_NEAR(1.7017, qn_oe.GetInclination_rad(), 1e-3);
  EXPECT_NEAR(5.9376, qn_oe.GetRaan_rad(), 1e-3);
  EXPECT_NEAR(3.6077, qn_oe.GetTrueLatAng_rad(), 1e-3);
  // Parameters
  EXPECT_NEAR(6.8993218223e6, qn_oe.GetSemiLatusRectum_m(), 1e-1);
  EXPECT_NEAR(r_norm_m, qn_oe.GetRadius_m(), 1e-1);
}

TEST(QuasiNonsingularOrbitalElements, Subtract) {
  // lhs
  const double lhs_semi_major_axis_m = 6896e3;
  const double lhs_eccentricity_x = 0.0;  // Test singular point
  const double lhs_eccentricity_y = 0.0;  // Test singular point
  const double lhs_inclination_rad = 1.7;
  const double lhs_raan_rad = 5.93;
  const double lhs_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements lhs_qn_oe(lhs_semi_major_axis_m, lhs_eccentricity_x, lhs_eccentricity_y, lhs_inclination_rad, lhs_raan_rad,
                                            lhs_true_latitude_angle_rad);
  // rhs
  const double rhs_semi_major_axis_m = 6896e3;
  const double rhs_eccentricity_x = 0.05;
  const double rhs_eccentricity_y = 0.03;
  const double rhs_inclination_rad = 1.7;
  const double rhs_raan_rad = 5.93;
  const double rhs_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements rhs_qn_oe(rhs_semi_major_axis_m, rhs_eccentricity_x, rhs_eccentricity_y, rhs_inclination_rad, rhs_raan_rad,
                                            rhs_true_latitude_angle_rad);

  QuasiNonsingularOrbitalElements qn_oe = lhs_qn_oe - rhs_qn_oe;
  // OEs
  EXPECT_NEAR(lhs_semi_major_axis_m - rhs_semi_major_axis_m, qn_oe.GetSemiMajor_m(), 1);
  EXPECT_NEAR(lhs_eccentricity_x - rhs_eccentricity_x, qn_oe.GetEccentricityX(), 1e-6);
  EXPECT_NEAR(lhs_eccentricity_y - rhs_eccentricity_y, qn_oe.GetEccentricityY(), 1e-6);
  EXPECT_NEAR(lhs_inclination_rad - rhs_inclination_rad, qn_oe.GetInclination_rad(), 1e-3);
  EXPECT_NEAR(lhs_raan_rad - rhs_raan_rad, qn_oe.GetRaan_rad(), 1e-3);
  EXPECT_NEAR(lhs_true_latitude_angle_rad - rhs_true_latitude_angle_rad, qn_oe.GetTrueLatAng_rad(), 1e-3);
}