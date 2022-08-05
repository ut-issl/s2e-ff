#include <gtest/gtest.h>

#include "../src/Library/math/DualQuaternion.hpp"

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
  libra::Quaternion q_real(0, 0, 0, 1);
  libra::Quaternion q_dual(0, 0, 0, 2);
  libra::DualQuaternion dq(q_real, q_dual);

  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq.GetDualPart()[3]);
}

TEST(DualQuaternion, ConstructorFromEightValue) {
  libra::DualQuaternion dq(0, 0, 0, 2, 0, 0, 0, 1);

  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetDualPart()[3]);
}

TEST(DualQuaternion, ConstructorFromRotationTranslation) {
  libra::Quaternion q_rot(1.0, 0.0, 0.0, 1.0);  // 90deg rotation around X axis
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 2.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[3]);

  libra::Vector<3> v_out = dq.GetTranslationVector();
  EXPECT_DOUBLE_EQ(v_translation[0], v_out[0]);
  EXPECT_DOUBLE_EQ(v_translation[1], v_out[1]);
  EXPECT_DOUBLE_EQ(v_translation[2], v_out[2]);
}

TEST(DualQuaternion, Normalize) {
  libra::DualQuaternion dq(0, 0, 0, 2, 0, 0, 1, 0);
  libra::DualQuaternion dq_out = dq.CalcNormalizedRotationQauternion();

  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[3]);

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[3]);

  dq.NormalizeRotationQauternion();
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(2.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[3]);
}

TEST(DualQuaternion, DualNumberConjugate) {
  libra::DualQuaternion dq(0, 0, 0, 1, 0, 0, 0, 1);
  libra::DualQuaternion dq_out = dq.DualNumberConjugate();

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(-1.0, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, QuaternionConjugate) {
  libra::DualQuaternion dq(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
  libra::DualQuaternion dq_out = dq.QuaternionConjugate();

  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, DualQuaternionConjugate) {
  libra::DualQuaternion dq(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
  libra::DualQuaternion dq_out = dq.DualQuaternionConjugate();

  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.5, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(-0.5, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, TransformVectorXrot) {
  libra::Quaternion q_rot(1.0, 0.0, 0.0, 1.0);  // 90deg rotation around X axis
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 1.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_DOUBLE_EQ(1.0, v_out[0]);
  EXPECT_DOUBLE_EQ(0.0, v_out[1]);
  EXPECT_DOUBLE_EQ(1.0, v_out[2]);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-9);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-9);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-9);
}

TEST(DualQuaternion, TransformVectorYrot) {
  libra::Quaternion q_rot(0.0, 1.0, 0.0, 1.0);  // 90deg rotation around Y axis
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 2.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_DOUBLE_EQ(0.0, v_out[0]);
  EXPECT_DOUBLE_EQ(0.0, v_out[1]);
  EXPECT_DOUBLE_EQ(1.0, v_out[2]);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-9);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-9);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-9);
}

TEST(DualQuaternion, TransformVectorZrot) {
  libra::Quaternion q_rot(0.0, 0.0, 1.0, 1.0);  // 90deg rotation around Z axis
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 1.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_DOUBLE_EQ(0.0, v_out[0]);
  EXPECT_DOUBLE_EQ(1.0, v_out[1]);
  EXPECT_DOUBLE_EQ(1.0, v_out[2]);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-9);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-9);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-9);
}

TEST(DualQuaternion, TransformVectorAllAxes) {
  libra::Quaternion q_rot(1.0, 1.0, 1.0, 1.0);
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = -1.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_DOUBLE_EQ(0.0, v_out[0]);
  EXPECT_DOUBLE_EQ(1.0, v_out[1]);
  EXPECT_DOUBLE_EQ(-1.0, v_out[2]);

  libra::Vector<3> v_out_inverse = dq.InverseTransformVector(v_out);

  EXPECT_NEAR(v_in[0], v_out_inverse[0], 1e-9);
  EXPECT_NEAR(v_in[1], v_out_inverse[1], 1e-9);
  EXPECT_NEAR(v_in[2], v_out_inverse[2], 1e-9);
}

TEST(DualQuaternion, Integral) {
  libra::DualQuaternion dq(0, 0, 0, 1, 0, 0, 0, 0);
  libra::Vector<3> omega;
  omega[0] = 0.01745329;
  omega[1] = 0.0;
  omega[2] = 0.0;
  libra::Vector<3> velocity;
  velocity[0] = 1.0;
  velocity[1] = 0.0;
  velocity[2] = 0.0;
  double dt = 1.0;

  for (int i = 0; i < 90; i++) {
    dq = dq.Integrate(omega, velocity, dt);
  }

  EXPECT_NEAR(0.7071, dq.GetRealPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq.GetRealPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq.GetRealPart()[2], 1e-3);
  EXPECT_NEAR(0.7071, dq.GetRealPart()[3], 1e-3);
  EXPECT_NEAR(31.823, dq.GetDualPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq.GetDualPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq.GetDualPart()[2], 1e-3);
  EXPECT_NEAR(-31.822, dq.GetDualPart()[3], 1e-3);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 1.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(91.0, v_out[0], 1e-2);
  EXPECT_NEAR(0.0, v_out[1], 1e-2);
  EXPECT_NEAR(1.0, v_out[2], 1e-2);
}

TEST(DualQuaternion, SclerpTranslationOnly) {
  libra::DualQuaternion dq1(0, 0, 0, 1, 0, 0, 0, 0);
  libra::DualQuaternion dq2(0, 0, 0, 1, 1, 0, 0, 0);

  libra::DualQuaternion dq_out = libra::Sclerp(dq1, dq2, 0.5);

  EXPECT_NEAR(0.0, dq_out.GetRealPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[2], 1e-3);
  EXPECT_NEAR(1.0, dq_out.GetRealPart()[3], 1e-3);
  EXPECT_NEAR(0.5, dq_out.GetDualPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetDualPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetDualPart()[2], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetDualPart()[3], 1e-3);
}

TEST(DualQuaternion, Sclerp) {
  libra::DualQuaternion dq1(0, 0, 0, 1, 0, 0, 0, 0);
  // X 90deg rotation and X axis translation
  libra::Quaternion q2_rot(1.0, 0.0, 0.0, 1.0);
  libra::Vector<3> v2_translation;
  v2_translation[0] = 1.0;
  v2_translation[1] = 0.0;
  v2_translation[2] = 0.0;
  libra::DualQuaternion dq2(q2_rot, v2_translation);

  // X 45deg rotation and X axis harf translation
  libra::DualQuaternion dq_out = libra::Sclerp(dq1, dq2, 0.5);

  EXPECT_NEAR(0.3826, dq_out.GetRealPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[2], 1e-3);
  EXPECT_NEAR(0.9239, dq_out.GetRealPart()[3], 1e-3);
  EXPECT_NEAR(0.23097, dq_out.GetDualPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetDualPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetDualPart()[2], 1e-3);
  EXPECT_NEAR(-0.0957, dq_out.GetDualPart()[3], 1e-3);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 1.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq_out.TransformVector(v_in);
  EXPECT_NEAR(1.5, v_out[0], 1e-3);
  EXPECT_NEAR(0.7071, v_out[1], 1e-3);
  EXPECT_NEAR(0.7071, v_out[2], 1e-3);
}

TEST(DualQuaternion, Addition) {
  libra::DualQuaternion dq_lhs(0, 0, 0, 1, 0, 0, 0, 1);
  libra::DualQuaternion dq_rhs(0, 0, 0, 1, 0, 0, 0, 1);
  libra::DualQuaternion dq_out = dq_lhs + dq_rhs;

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, Subtraction) {
  libra::DualQuaternion dq_lhs(0, 0, 0, 3, 0, 0, 0, 3);
  libra::DualQuaternion dq_rhs(0, 0, 0, 1, 0, 0, 0, 1);
  libra::DualQuaternion dq_out = dq_lhs - dq_rhs;

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[3]);
}

TEST(DualQuaternion, ScalarProduction) {
  libra::DualQuaternion dq(0, 0, 0, 1, 0, 0, 0, 1);
  double scalar = 2.0;
  libra::DualQuaternion dq_out = scalar * dq;

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[3]);
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
