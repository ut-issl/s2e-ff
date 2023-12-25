/**
 * @file corner_cube_reflector.cpp
 * @brief Corner cube reflector
 */

#include "corner_cube_reflector.hpp"

CornerCubeReflector::CornerCubeReflector(const int prescaler, ClockGenerator* clock_gen, const Dynamics* dynamics)
    : Component(prescaler, clock_gen), dynamics_(dynamics) {
  normal_direction_c_[0] = 0.0;
  normal_direction_c_[1] = 0.0;
  normal_direction_c_[2] = 1.0;
  reflectable_angle_rad_ = 0.1;

  libra::Quaternion q_rot(0.0, 1.0, 0.0, 1.0);
  libra::Vector<3> v_translation_m;
  v_translation_m[0] = 0.5;
  v_translation_m[1] = 0.0;
  v_translation_m[2] = 0.0;
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-v_translation_m, q_rot.Conjugate()).QuaternionConjugate();
}

void CornerCubeReflector::MainRoutine(int count) {
  if (count < 10) return;

  // Body -> Inertial frame
  libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
  libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b().Conjugate();
  libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b);

  libra::Vector<3> position_i_m = dual_quaternion_i2b.InverseTransformVector(libra::Vector<3>{0.0});

  // Component -> Inertial frame
  libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

  reflector_position_i_m_ = dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
  normal_direction_i_ = dual_quaternion_c2i.TransformVector(normal_direction_c_);
  normal_direction_i_ -= reflector_position_i_m_;
}

std::string CornerCubeReflector::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteVector("normal_direction", "i", "", 3);

  return str_tmp;
}

std::string CornerCubeReflector::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(normal_direction_i_);

  return str_tmp;
}
