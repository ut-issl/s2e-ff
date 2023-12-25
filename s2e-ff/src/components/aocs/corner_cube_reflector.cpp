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

  libra::Quaternion q_b2c(0.0, 1.0, 0.0, 1.0);
  libra::Vector<3> position_b2c_m;
  position_b2c_m[0] = 0.5;
  position_b2c_m[1] = 0.0;
  position_b2c_m[2] = 0.0;
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b2c_m, q_b2c.Conjugate()).QuaternionConjugate();
}

void CornerCubeReflector::MainRoutine(int count) { UNUSED(count); }

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
