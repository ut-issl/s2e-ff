/**
 * @file laser_emitter.cpp
 * @brief Laser Emitter
 */
#include "./laser_emitter.hpp"

/**
 * @fn LaserEmitter
 * @brief Constructor
 */
LaserEmitter::LaserEmitter(const int prescaler, ClockGenerator* clock_gen, const Dynamics& dynamics, libra::Vector<3> emitting_direction_c,
                           double emission_angle_rad, libra::TranslationFirstDualQuaternion dual_quaternion_c2b, double emission_power_W,
                           double radius_beam_waist_m, double rayleigh_length_m, double rayleigh_length_offset_m, double wavelength_m)
    : Component(prescaler, clock_gen),
      GaussianBeamBase(wavelength_m, radius_beam_waist_m, emission_power_W),
      dynamics_(dynamics),
      emitting_direction_c_(emitting_direction_c),
      emission_angle_rad_(emission_angle_rad),
      dual_quaternion_c2b_(dual_quaternion_c2b),
      rayleigh_length_m_(rayleigh_length_m),
      rayleigh_length_offset_m_(rayleigh_length_offset_m) {}

void LaserEmitter::MainRoutine(int count) { Update(count); }

void LaserEmitter::Update(int count) {
  if (count_ >= count) return;
  // Body -> Inertial frame
  libra::Vector<3> spacecraft_position_i2b_m = dynamics_.GetOrbit().GetPosition_i_m();
  libra::Quaternion spacecraft_attitude_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
  libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

  // Component -> Inertial frame
  libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

  laser_position_i_m_ = dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});

  laser_emitting_direction_i_ = dual_quaternion_c2i.TransformVector(emitting_direction_c_) - laser_position_i_m_;

  count_ = count;
}

std::string LaserEmitter::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string LaserEmitter::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}

LaserEmitter InitializeLaserEmitter(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics& dynamics,
                                    const size_t id) {
  IniAccess ini_file(file_name);
  std::string name = "LASER_EMITTER_";
  const std::string section_name = name + std::to_string(static_cast<long long>(id));

  libra::Quaternion quaternion_b2c;
  ini_file.ReadQuaternion(section_name.c_str(), "quaternion_b2c", quaternion_b2c);
  libra::Vector<3> position_b_m;
  ini_file.ReadVector(section_name.c_str(), "position_b_m", position_b_m);
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b =
      libra::TranslationFirstDualQuaternion(-position_b_m, quaternion_b2c.Conjugate()).QuaternionConjugate();

  libra::Vector<3> emitting_direction_c{0.0};
  ini_file.ReadVector(section_name.c_str(), "emitting_direction_c", emitting_direction_c);

  double emission_angle_rad = ini_file.ReadDouble(section_name.c_str(), "emission_angle_rad");
  double emission_power_W = ini_file.ReadDouble(section_name.c_str(), "emission_power_W");

  double radius_beam_waist_m = ini_file.ReadDouble(section_name.c_str(), "radius_beam_waist_m");
  double rayleigh_length_m = ini_file.ReadDouble(section_name.c_str(), "rayleigh_length_m");
  double rayleigh_length_offset_m = ini_file.ReadDouble(section_name.c_str(), "rayleigh_length_offset_m");

  double wavelength_m = libra::pi * radius_beam_waist_m * radius_beam_waist_m / rayleigh_length_m;

  LaserEmitter laser_emitter(prescaler, clock_gen, dynamics, emitting_direction_c, emission_angle_rad, dual_quaternion_c2b, emission_power_W,
                             radius_beam_waist_m, rayleigh_length_m, rayleigh_length_offset_m, wavelength_m);

  return laser_emitter;
};
