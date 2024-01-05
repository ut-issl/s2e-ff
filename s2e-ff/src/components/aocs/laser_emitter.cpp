/**
 * @file laser_emitter.cpp
 * @brief Laser Emitter
 */
#include "./laser_emitter.hpp"

/**
 * @fn LaserEmitter
 * @brief Constructor
 */
LaserEmitter::LaserEmitter(const Dynamics& dynamics, libra::Vector<3> emitting_direction_c, double emission_angle_rad,
                           libra::TranslationFirstDualQuaternion dual_quaternion_c2b, double emission_power_W, double radius_beam_waist_m,
                           double rayleigh_length_m, double rayleigh_length_offset_m, double wavelength_m)
    : GaussianBeamBase(wavelength_m, radius_beam_waist_m, emission_power_W),
      dynamics_(dynamics),
      emitting_direction_c_(emitting_direction_c),
      emission_angle_rad_(emission_angle_rad),
      dual_quaternion_c2b_(dual_quaternion_c2b),
      rayleigh_length_m_(rayleigh_length_m),
      rayleigh_length_offset_m_(rayleigh_length_offset_m) {}

double LaserEmitter::CalcBeamWidthRadius_m(const double emission_distance_m) {
  const double radius_beam_waist_m = GetBeamWaistRadius_m();
  double beam_width_radius_m = radius_beam_waist_m * sqrt(1.0 + pow(((emission_distance_m - rayleigh_length_offset_m_) / rayleigh_length_m_), 2.0));
  return beam_width_radius_m;
}

double LaserEmitter::CalcIntensity_W_m2(double emission_distance_m, double deviation_from_optical_axis_m) {
  double beam_width_radius_m = CalcBeamWidthRadius_m(emission_distance_m);
  double peak_intensity_W_m2 = (2.0 * GetTotalPower_W()) / (libra::pi * beam_width_radius_m * beam_width_radius_m);
  double gaussian_dist =
      std::exp((-2.0 * deviation_from_optical_axis_m * deviation_from_optical_axis_m) / (beam_width_radius_m * beam_width_radius_m));
  double intensity_W_m2 = peak_intensity_W_m2 * gaussian_dist;
  return intensity_W_m2;
}

LaserEmitter InitializeLaserEmitter(const std::string file_name, const Dynamics& dynamics, const size_t id) {
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

  LaserEmitter laser_emitter(dynamics, emitting_direction_c, emission_angle_rad, dual_quaternion_c2b, emission_power_W, radius_beam_waist_m,
                             rayleigh_length_m, rayleigh_length_offset_m, wavelength_m);

  return laser_emitter;
};
