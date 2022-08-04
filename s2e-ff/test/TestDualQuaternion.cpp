#include <gtest/gtest.h>

#include "../src/Library/math/DualQuaternion.hpp"

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
  libra::Quaternion q_rot(0, 0, 0, 1);
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 2.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[3]);

  libra::Vector<3> v_out = dq.GetTranslationVector();
  EXPECT_DOUBLE_EQ(0.0, v_out[0]);
  EXPECT_DOUBLE_EQ(0.0, v_out[1]);
  EXPECT_DOUBLE_EQ(2.0, v_out[2]);
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

TEST(DualQuaternion, ConvertFrame) {
  libra::Quaternion q_rot(0.0, 0.0, 0.0, 1.0);
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 1.0;
  libra::DualQuaternion dq(q_rot, v_translation);

  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq.ConvertFrame(v_in);

  EXPECT_DOUBLE_EQ(1.0, v_out[0]);
  EXPECT_DOUBLE_EQ(0.0, v_out[1]);
  EXPECT_DOUBLE_EQ(1.0, v_out[2]);
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
