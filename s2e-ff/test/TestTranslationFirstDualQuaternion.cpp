#include <gtest/gtest.h>

#include "../src/Library/math/TranslationFirstDualQuaternion.hpp"

TEST(TranslationFirstDualQuaternion, ConstructorFromRotationTranslation) {
  libra::Quaternion q_rot(1.0, 0.0, 0.0, 1.0);  // 90deg rotation around X axis
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 0.0;
  v_translation[2] = 2.0;
  libra::TranslationFirstDualQuaternion dq(v_translation, q_rot);

  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(-1.0 / sqrt(2.0), dq.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(1.0 / sqrt(2.0), dq.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq.GetDualPart()[3]);

  libra::Vector<3> v_out = dq.GetTranslationVector();
  EXPECT_DOUBLE_EQ(v_translation[0], v_out[0]);
  EXPECT_DOUBLE_EQ(v_translation[1], v_out[1]);
  EXPECT_DOUBLE_EQ(v_translation[2], v_out[2]);
}

TEST(TranslationFirstDualQuaternion, Normalize) {
  libra::DualQuaternion dq(0, 0, 0, 2, 0, 0, 1, 0);
  libra::TranslationFirstDualQuaternion dq_in(dq);
  libra::TranslationFirstDualQuaternion dq_out = dq_in.CalcNormalizedRotationQauternion();

  EXPECT_DOUBLE_EQ(0.0, dq_in.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(2.0, dq_in.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(1.0, dq_in.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetDualPart()[3]);

  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq_out.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(2.0, dq_out.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq_out.GetDualPart()[3]);

  dq_in.NormalizeRotationQauternion();
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetRealPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetRealPart()[1]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetRealPart()[2]);
  EXPECT_DOUBLE_EQ(1.0, dq_in.GetRealPart()[3]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetDualPart()[0]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetDualPart()[1]);
  EXPECT_DOUBLE_EQ(2.0, dq_in.GetDualPart()[2]);
  EXPECT_DOUBLE_EQ(0.0, dq_in.GetDualPart()[3]);
}

TEST(TranslationFirstDualQuaternion, Integral) {
  // Initial DualQuaternion
  libra::Quaternion q_rot(0.0, 0.0, 0.0, 1.0);
  libra::Vector<3> v_translation;
  v_translation[0] = 0.0;
  v_translation[1] = 1.0;
  v_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq(v_translation, q_rot);

  // Kinematics condition
  libra::Vector<3> omega;
  omega[0] = 0.01745329;  // 1 deg/s
  omega[1] = 0.0;
  omega[2] = 0.0;
  libra::Vector<3> velocity;
  velocity[0] = 1.0;
  velocity[1] = 0.0;
  velocity[2] = 0.0;
  double dt = 0.1;

  // Integration
  for (int i = 0; i < 900; i++) {
    dq = dq.Integrate(omega, velocity, dt);
  }

  // Check rotation
  EXPECT_NEAR(0.7071, dq.GetRealPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq.GetRealPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq.GetRealPart()[2], 1e-3);
  EXPECT_NEAR(0.7071, dq.GetRealPart()[3], 1e-3);

  // Check transition
  EXPECT_NEAR(90.0, dq.GetTranslationVector()[0], 1e-1);
  EXPECT_NEAR(1.0, dq.GetTranslationVector()[1], 1e-1);
  EXPECT_NEAR(0.0, dq.GetTranslationVector()[2], 1e-1);

  // Check frame conversion
  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 1.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(91.0, v_out[0], 1e-1);
  EXPECT_NEAR(-1.0, v_out[1], 1e-1);
  EXPECT_NEAR(1.0, v_out[2], 1e-1);
}

TEST(TranslationFirstDualQuaternion, SclerpTranslationOnly) {
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

TEST(TranslationFirstDualQuaternion, Sclerp) {
  libra::TranslationFirstDualQuaternion dq1;
  // X 90deg rotation and X axis translation
  libra::Quaternion q2_rot(1.0, 0.0, 0.0, 1.0);
  libra::Vector<3> v2_translation;
  v2_translation[0] = 1.0;
  v2_translation[1] = 0.0;
  v2_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq2(v2_translation, q2_rot);

  // X 45deg rotation and X axis harf translation
  libra::TranslationFirstDualQuaternion dq_out = libra::Sclerp(dq1, dq2, 0.5);

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
