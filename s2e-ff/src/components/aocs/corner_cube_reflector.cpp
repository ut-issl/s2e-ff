/**
 * @file corner_cube_reflector.cpp
 * @brief Corner cube reflector
 */

#include "./corner_cube_reflector.hpp"

CornerCubeReflector::CornerCubeReflector(const int prescaler, ClockGenerator* clock_gen) : Component(prescaler, clock_gen), dynamics_(nullptr){};

CornerCubeReflector::CornerCubeReflector(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics,
                                         const size_t id)
    : Component(prescaler, clock_gen), dynamics_(dynamics) {
  Initialize(file_name, id);
}

void CornerCubeReflector::MainRoutine(int count) { Update(count); }

void CornerCubeReflector::Update(int count) {
  if (count_ >= count) return;
  // Body -> Inertial frame
  libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
  libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

  // Component -> Inertial frame
  libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

  corner_cube_reflector_position_i_m_ = dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
  corner_cube_reflector_normal_direction_i_ = dual_quaternion_c2i.TransformVector(normal_direction_c_) - corner_cube_reflector_position_i_m_;

  count_ = count;
}

std::string CornerCubeReflector::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string CornerCubeReflector::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}

void CornerCubeReflector::Initialize(const std::string file_name, const size_t id) {
  IniAccess ini_file(file_name);
  std::string name = "CORNER_CUBE_REFLECTOR_";
  const std::string section_name = name + std::to_string(static_cast<long long>(id));

  libra::Quaternion quaternion_b2c;
  ini_file.ReadQuaternion(section_name.c_str(), "quaternion_b2c", quaternion_b2c);
  libra::Vector<3> position_b_m;
  ini_file.ReadVector(section_name.c_str(), "position_b_m", position_b_m);
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b_m, quaternion_b2c.Conjugate()).QuaternionConjugate();

  ini_file.ReadVector(section_name.c_str(), "normal_direction_c", normal_direction_c_);
  reflectable_angle_rad_ = ini_file.ReadDouble(section_name.c_str(), "reflectable_angle_rad");
}