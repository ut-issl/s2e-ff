#include <gtest/gtest.h>

#include "../src/Library/math/DualQuaternion.hpp"

TEST(DualQuaternion, Constructor) {
  libra::DualQuaternion dq(0, 0, 0, 1, 0, 0, 0, 1);

  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq.GetDualPart()[3]);
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
