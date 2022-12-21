#include <gtest/gtest.h>

#include "../src/Library/Orbit/QuasiNonsingularOrbitalElements.hpp"

TEST(QuasiNonsingularOrbitalElement, DefaultConstructor) {
  QuasiNonsingularOrbitalElements qn_oe;

  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetSemiMajor_m());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetEccentricityX());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetEccentricityY());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetInclination_rad());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetRaan_rad());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetTrueLatAng_rad());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetSemiLatusRectum_m());
  EXPECT_DOUBLE_EQ(0.0, qn_oe.GetRadius_m());
}
