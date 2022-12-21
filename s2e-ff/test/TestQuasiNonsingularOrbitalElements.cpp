#include <gtest/gtest.h>

#include "../src/Library/Orbit/QuasiNonsingularOrbitalElements.hpp"

TEST(QuasiNonsingularOrbitalElement, DefaultConstructor) {
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

TEST(QuasiNonsingularOrbitalElement, ConstructorWithSingularOE) {
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

TEST(QuasiNonsingularOrbitalElement, ConstructorWithOE) {
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

TEST(QuasiNonsingularOrbitalElement, ConstructorWithPositionVelocity) {
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
