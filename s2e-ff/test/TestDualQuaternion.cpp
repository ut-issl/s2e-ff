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

/*
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
*/

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
