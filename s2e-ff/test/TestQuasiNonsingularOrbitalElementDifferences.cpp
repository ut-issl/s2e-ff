#include <gtest/gtest.h>

#include "../src/Library/RelativeOrbit/QuasiNonsingularOrbitalElementDifferences.hpp"

TEST(QuasiNonsingularOrbitalElementDifferences, ConstructorWithOe) {
  // lhs
  const double reference_semi_major_axis_m = 6896e3;
  const double reference_eccentricity_x = 0.0;  // Test singular point
  const double reference_eccentricity_y = 0.0;  // Test singular point
  const double reference_inclination_rad = 1.7;
  const double reference_raan_rad = 5.93;
  const double reference_true_latitude_angle_rad = 0.5;
  QuasiNonsingularOrbitalElements reference_qn_oe(reference_semi_major_axis_m, reference_eccentricity_x, reference_eccentricity_y,
                                                  reference_inclination_rad, reference_raan_rad, reference_true_latitude_angle_rad);
  // rhs
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
