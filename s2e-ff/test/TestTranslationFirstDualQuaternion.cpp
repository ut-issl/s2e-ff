#include <gtest/gtest.h>

#include "../src/Library/math/TranslationFirstDualQuaternion.hpp"
#include <iostream>

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

  // Check vector transform
  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 0.0;
  v_in[2] = 1.0;

  libra::Vector<3> v_conv = dq.TransformVector(v_in);

  EXPECT_NEAR(1.0, v_conv[0], 1e-1);
  EXPECT_NEAR(-3.0, v_conv[1], 1e-1);
  EXPECT_NEAR(0.0, v_conv[2], 1e-1);
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
  double dt = 0.01;

  // Integration
  //std::cout << "time[sec],";
  //std::cout << "dq_r_x, dq_r_y, dq_r_z, dq_r_w,";
  //std::cout << "dq_d_x, dq_d_y, dq_d_z, dq_d_w,";
  //std::cout << "v_x, v_y, v_z,";
  //std::cout << std::endl;
  for (int i = 0; i < 9000; i++) {
    dq = dq.Integrate(omega, velocity, dt);
    //std::cout << i * dt << ",";
    //std::cout << dq.GetRealPart()[0] << "," << dq.GetRealPart()[1] << "," << dq.GetRealPart()[2] << "," << dq.GetRealPart()[3] << ",";
    //std::cout << dq.GetDualPart()[0] << "," << dq.GetDualPart()[1] << "," << dq.GetDualPart()[2] << "," << dq.GetDualPart()[3] << ",";
    //std::cout << dq.GetTranslationVector()[0] << "," << dq.GetTranslationVector()[1] << "," << dq.GetTranslationVector()[2] << ",";
    //std::cout << std::endl;
  }

  // Check rotation
  EXPECT_NEAR(0.7071, dq.GetRealPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq.GetRealPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq.GetRealPart()[2], 1e-3);
  EXPECT_NEAR(0.7071, dq.GetRealPart()[3], 1e-3);

  // Check transition
  EXPECT_NEAR(90.0, dq.GetTranslationVector()[0], 1e-2);
  EXPECT_NEAR(1.0, dq.GetTranslationVector()[1], 1e-2);
  EXPECT_NEAR(0.0, dq.GetTranslationVector()[2], 1e-2);

  // Check vector transform
  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 1.0;
  v_in[2] = 1.0;

  libra::Vector<3> v_out = dq.TransformVector(v_in);

  EXPECT_NEAR(91.0, v_out[0], 1e-2);
  EXPECT_NEAR(-1.0, v_out[1], 1e-2);
  EXPECT_NEAR(2.0, v_out[2], 1e-2);
}

TEST(TranslationFirstDualQuaternion, SclerpTranslationOnly) {
  // Initial DualQuaternion
  libra::Vector<3> q1_axis;
  q1_axis[0] = 1.0;
  q1_axis[1] = 0.0;
  q1_axis[2] = 0.0;
  libra::Quaternion q1_rot(q1_axis, 0.0);
  libra::Vector<3> v1_translation;
  v1_translation[0] = 0.0;
  v1_translation[1] = 0.0;
  v1_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq1(v1_translation, q1_rot);

  libra::Vector<3> q2_axis;
  q2_axis[0] = 1.0;
  q2_axis[1] = 0.0;
  q2_axis[2] = 0.0;
  libra::Quaternion q2_rot(q2_axis, 0.0);
  libra::Vector<3> v2_translation;
  v2_translation[0] = 1.0;
  v2_translation[1] = 1.0;
  v2_translation[2] = 1.0;
  libra::TranslationFirstDualQuaternion dq2(v2_translation, q2_rot);

  libra::TranslationFirstDualQuaternion dq_out = libra::Sclerp(dq1, dq2, 0.5);

  EXPECT_NEAR(0.0, dq_out.GetRealPart()[0], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[1], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[2], 1e-3);
  EXPECT_NEAR(1.0, dq_out.GetRealPart()[3], 1e-3);
  EXPECT_NEAR(0.25, dq_out.GetDualPart()[0], 1e-3);
  EXPECT_NEAR(0.25, dq_out.GetDualPart()[1], 1e-3);
  EXPECT_NEAR(0.25, dq_out.GetDualPart()[2], 1e-3);
  EXPECT_NEAR(0.0, dq_out.GetDualPart()[3], 1e-3);

  EXPECT_NEAR(0.5, dq_out.GetTranslationVector()[0], 1e-2);
  EXPECT_NEAR(0.5, dq_out.GetTranslationVector()[1], 1e-2);
  EXPECT_NEAR(0.5, dq_out.GetTranslationVector()[2], 1e-2);
}

TEST(TranslationFirstDualQuaternion, SclerpXAxisRotXAxisMove) {
  // Initial DualQuaternion
  libra::Vector<3> q1_axis;
  q1_axis[0] = 1.0;
  q1_axis[1] = 0.0;
  q1_axis[2] = 0.0;
  libra::Quaternion q1_rot(q1_axis, 0.0);
  libra::Vector<3> v1_translation;
  v1_translation[0] = 0.0;
  v1_translation[1] = 1.0;
  v1_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq1(v1_translation, q1_rot);

  // X 180deg rotation and X axis translation
  libra::Vector<3> q2_axis;
  q2_axis[0] = 1.0;
  q2_axis[1] = 0.0;
  q2_axis[2] = 0.0;
  libra::Quaternion q2_rot(q2_axis, 3.14159);
  libra::Vector<3> v2_translation;
  v2_translation[0] = 1.0;
  v2_translation[1] = 1.0;
  v2_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq2(v2_translation, q2_rot);

  // X 90deg rotation and X axis harf translation
  libra::TranslationFirstDualQuaternion dq_out = libra::Sclerp(dq1, dq2, 0.5);

  // Check rotation: 45 deg around X
  EXPECT_NEAR(0.7071, dq_out.GetRealPart()[0], 1e-2);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[1], 1e-2);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[2], 1e-2);
  EXPECT_NEAR(0.7071, dq_out.GetRealPart()[3], 1e-2);

  // Check translation: 0.5 move on X axis
  EXPECT_NEAR(0.5, dq_out.GetTranslationVector()[0], 1e-2);
  EXPECT_NEAR(1.0, dq_out.GetTranslationVector()[1], 1e-2);
  EXPECT_NEAR(0.0, dq_out.GetTranslationVector()[2], 1e-2);

  // Check vector transform
  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = 1.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq_out.TransformVector(v_in);
  EXPECT_NEAR(1.5, v_out[0], 1e-2);
  EXPECT_NEAR(0.0, v_out[1], 1e-2);
  EXPECT_NEAR(2.0, v_out[2], 1e-2);
}

TEST(TranslationFirstDualQuaternion, SclerpXAxisRotXYAxisMove) {
  // Initial DualQuaternion
  libra::Vector<3> q1_axis;
  q1_axis[0] = 1.0;
  q1_axis[1] = 0.0;
  q1_axis[2] = 0.0;
  libra::Quaternion q1_rot(q1_axis, 0.0);
  libra::Vector<3> v1_translation;
  v1_translation[0] = 0.0;
  v1_translation[1] = 1.0;
  v1_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq1(v1_translation, q1_rot);

  // X 180deg rotation and X axis translation
  libra::Vector<3> q2_axis;
  q2_axis[0] = 1.0;
  q2_axis[1] = 0.0;
  q2_axis[2] = 0.0;
  libra::Quaternion q2_rot(q2_axis, 3.14159);
  libra::Vector<3> v2_translation;
  v2_translation[0] = 2.0;
  v2_translation[1] = 2.0;
  v2_translation[2] = 0.0;
  libra::TranslationFirstDualQuaternion dq2(v2_translation, q2_rot);

  // debug
  std::cout << "time[sec],";
  std::cout << "dq_r_x, dq_r_y, dq_r_z, dq_r_w,";
  std::cout << "dq_d_x, dq_d_y, dq_d_z, dq_d_w,";
  std::cout << "v_x, v_y, v_z,";
  std::cout << std::endl;
  for (double duty = 0.005; duty <= 1; duty+=0.005)
  {
    libra::TranslationFirstDualQuaternion dq_test = libra::Sclerp(dq1, dq2, duty);
    std::cout << duty << ",";
    std::cout << dq_test.GetRealPart()[0] << "," << dq_test.GetRealPart()[1] << "," << dq_test.GetRealPart()[2] << "," << dq_test.GetRealPart()[3] << ",";
    std::cout << dq_test.GetDualPart()[0] << "," << dq_test.GetDualPart()[1] << "," << dq_test.GetDualPart()[2] << "," << dq_test.GetDualPart()[3] << ",";
    std::cout << dq_test.GetTranslationVector()[0] << "," << dq_test.GetTranslationVector()[1] << "," << dq_test.GetTranslationVector()[2] << ",";
    std::cout << std::endl;
  }
  // X 90deg rotation and X-Y axis translation
  libra::TranslationFirstDualQuaternion dq_out = libra::Sclerp(dq1, dq2, 0.5);

  // Check rotation: 45 deg around X
  EXPECT_NEAR(0.7071, dq_out.GetRealPart()[0], 1e-2);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[1], 1e-2);
  EXPECT_NEAR(0.0, dq_out.GetRealPart()[2], 1e-2);
  EXPECT_NEAR(0.7071, dq_out.GetRealPart()[3], 1e-2);

  // Check translation: 0.5 move on X axis
  EXPECT_NEAR(1.0, dq_out.GetTranslationVector()[0], 1e-2);
  EXPECT_NEAR(1.5, dq_out.GetTranslationVector()[1], 1e-2);
  EXPECT_NEAR(0.5, dq_out.GetTranslationVector()[2], 1e-2);

  // Check vector transform
  libra::Vector<3> v_in;
  v_in[0] = 1.0;
  v_in[1] = -1.0;
  v_in[2] = 0.0;

  libra::Vector<3> v_out = dq_out.TransformVector(v_in);
  EXPECT_NEAR(2.0, v_out[0], 1e-2);
  EXPECT_NEAR(-0.5, v_out[1], 1e-2);
  EXPECT_NEAR(0.5, v_out[2], 1e-2);
}
