#include <gtest/gtest.h>

#include "../src/Library/math/dual_quaternion.hpp"

TEST(DualQuaternion, DefaultConstructor) {
  libra::DualQuaternion dq;

  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[3]);
}

TEST(DualQuaternion, ConstructorFromTwoQuaternion) {
  libra::Quaternion q_real(0.5, 0.5, 0.5, 0.5);
  libra::Quaternion q_dual(0.1, 0.2, 0.3, 2);
  libra::DualQuaternion dq(q_real, q_dual);

  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.1, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.2, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.3, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq.GetDualPart()[3]);
}

TEST(DualQuaternion, ConstructorFromEightValue) {
  libra::DualQuaternion dq(0.5, 0.5, 0.5, 0.5, -0.1, -0.2, -0.3, -2);

  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(-0.1, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(-0.2, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(-0.3, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(-2.0, dq.GetDualPart()[3]);
}

TEST(DualQuaternion, DualNumberConjugate) {
  libra::DualQuaternion dq(0.5, 0.5, 0.5, 0.5, -0.1, -0.2, -0.3, -2);
  libra::DualQuaternion dq_out = dq.DualNumberConjugate();

  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.1, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.2, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.3, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, QuaternionConjugate) {
  libra::DualQuaternion dq(0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5);
  libra::DualQuaternion dq_out = dq.QuaternionConjugate();

  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, DualQuaternionConjugate) {
  libra::DualQuaternion dq(0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5);
  libra::DualQuaternion dq_out = dq.DualQuaternionConjugate();

  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, Inverse) {
  libra::DualQuaternion dq(0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5);
  libra::DualQuaternion dq_out = dq.Inverse();

  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, TransformVectorXrot) {
  libra::Quaternion q_real(0.707106, 0.0, 0.0, 0.707106);  // 90deg rotation around X axis
  libra::Quaternion q_dual(0.0, 0.707106, 0.707106, 0.0);  // 2 step move to Z axis
  libra::DualQuaternion dq(q_real, q_dual);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(1.0, v_out[0], 1e-4);
  EXPECT_NEAR(0.0, v_out[1], 1e-4);
  EXPECT_NEAR(2.0, v_out[2], 1e-4);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-4);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-4);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-4);
}

TEST(DualQuaternion, TransformVectorYrot) {
  libra::Quaternion q_real(0.0, 0.707106, 0.0, 0.707106);   // 90deg rotation around Y axis
  libra::Quaternion q_dual(-0.707106, 0.0, 0.707106, 0.0);  // 2 step move to Z axis
  libra::DualQuaternion dq(q_real, q_dual);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(0.0, v_out[0], 1e-4);
  EXPECT_NEAR(0.0, v_out[1], 1e-4);
  EXPECT_NEAR(1.0, v_out[2], 1e-4);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-4);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-4);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-4);
}

TEST(DualQuaternion, TransformVectorZrot) {
  libra::Quaternion q_real(0.0, 0.0, 0.707106, 0.707106);   // 90deg rotation around Z axis
  libra::Quaternion q_dual(0.0, 0.0, 0.707106, -0.707106);  // 2 step move to Z axis
  libra::DualQuaternion dq(q_real, q_dual);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(0.0, v_out[0], 1e-4);
  EXPECT_NEAR(1.0, v_out[1], 1e-4);
  EXPECT_NEAR(2.0, v_out[2], 1e-4);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-4);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-4);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-4);
}

TEST(DualQuaternion, TransformVectorAllAxes) {
  libra::Quaternion q_real(0.5, 0.5, 0.5, 0.5);        // all axis rotation
  libra::Quaternion q_dual(-0.25, 0.25, 0.25, -0.25);  // 1 step move to Z axis
  libra::DualQuaternion dq(q_real, q_dual);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(0.0, v_out[0], 1e-4);
  EXPECT_NEAR(1.0, v_out[1], 1e-4);
  EXPECT_NEAR(1.0, v_out[2], 1e-4);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-4);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-4);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-4);
}

TEST(DualQuaternion, CalcScrewParametersNoRotation) {
  // No rotation error
  libra::DualQuaternion dq(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  libra::ScrewParameters screw = dq.CalcScrewParameters();

  EXPECT_DOUBLE_EQ(0.0, screw.angle_rad_);
  EXPECT_DOUBLE_EQ(0.0, screw.pitch_);
  EXPECT_DOUBLE_EQ(0.0, screw.axis_[0]);
  EXPECT_DOUBLE_EQ(0.0, screw.axis_[1]);
  EXPECT_DOUBLE_EQ(0.0, screw.axis_[2]);
  EXPECT_DOUBLE_EQ(0.0, screw.moment_[0]);
  EXPECT_DOUBLE_EQ(0.0, screw.moment_[1]);
  EXPECT_DOUBLE_EQ(0.0, screw.moment_[2]);
}

TEST(DualQuaternion, CalcScrewParameters) {
  libra::DualQuaternion dq(0.707, 0.0, 0.0, 0.707, 0.5, 0.5, 0.5, 0.5);
  libra::ScrewParameters screw = dq.CalcScrewParameters();

  EXPECT_NEAR(1.5707, screw.angle_rad_, 1e-3);
  EXPECT_NEAR(-1.4144, screw.pitch_, 1e-3);
  EXPECT_NEAR(1.0, screw.axis_[0], 1e-3);
  EXPECT_NEAR(0.0, screw.axis_[1], 1e-3);
  EXPECT_NEAR(0.0, screw.axis_[2], 1e-3);
  EXPECT_NEAR(1.4144, screw.moment_[0], 1e-3);
  EXPECT_NEAR(0.7072, screw.moment_[1], 1e-3);
  EXPECT_NEAR(0.7072, screw.moment_[2], 1e-3);
}

TEST(DualQuaternion, Addition) {
  libra::DualQuaternion dq_lhs(0.1, 0.2, 0.3, 1, 0.4, 0.5, 0.6, 1);
  libra::DualQuaternion dq_rhs(-0.7, 0.8, 0.9, 1, 0.1, -0.2, 0.3, 1);
  libra::DualQuaternion dq_out = dq_lhs + dq_rhs;

  EXPECT_DOUBLE_EQ(-0.6, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(1.2, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.3, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.9, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, Subtraction) {
  libra::DualQuaternion dq_lhs(0.1, 0.2, 0.3, 0.5, 0.4, 0.5, 0.6, 1);
  libra::DualQuaternion dq_rhs(-0.7, 0.8, 0.9, 1, 0.1, -0.2, 0.3, 0.5);
  libra::DualQuaternion dq_out = dq_lhs - dq_rhs;

  EXPECT_DOUBLE_EQ(0.8, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(-0.6, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(-0.6, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.3, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.7, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.3, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, ScalarProduction) {
  libra::DualQuaternion dq(0.5, 0.5, 0.5, 0.5, -0.1, -0.2, -0.3, -2);
  double scalar = 2.0;
  libra::DualQuaternion dq_out = scalar * dq;

  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(-0.2, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(-0.4, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(-0.6, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(-4.0, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, DualQuaternionProduction) {
  libra::DualQuaternion dq_lhs(0, 0, 0, 1, 0, 0, 0, 1);
  libra::DualQuaternion dq_rhs(0, 0, 0, 1, 0, 0, 0, 1);
  libra::DualQuaternion dq_out = dq_lhs * dq_rhs;

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[3]);
}
