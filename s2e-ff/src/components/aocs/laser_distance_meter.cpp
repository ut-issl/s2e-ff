/**
 * @file laser_distance_meter.hpp
 * @brief Laser distance meter
 */

#include "laser_distance_meter.hpp"

LaserDistanceMeter::LaserDistanceMeter(const int prescaler, ClockGenerator* clock_gen, const Dynamics& dynamics,
                                       const FfInterSpacecraftCommunication& inter_spacecraft_communication)
    : Component(prescaler, clock_gen), dynamics_(dynamics), inter_spacecraft_communication_(inter_spacecraft_communication) {
  laser_emitting_direction_c_[0] = -1.0;
  laser_emitting_direction_c_[1] = 0.0;
  laser_emitting_direction_c_[2] = 0.0;
  emission_angle_rad_ = 0.1;

  libra::Quaternion q_b2c(0.0, 0.0, 0.0, 1.0);
  libra::Vector<3> position_b2c_m;
  position_b2c_m[0] = -0.5;
  position_b2c_m[1] = 0.0;
  position_b2c_m[2] = 0.0;
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b2c_m, q_b2c);

  laser_emission_position_b_m_ = dual_quaternion_c2b_.TransformVector(libra::Vector<3>{0.0});
  laser_emitting_direction_b_ = dual_quaternion_c2b_.TransformVector(laser_emitting_direction_c_);
  laser_emitting_direction_b_ -= laser_emission_position_b_m_;
}

void LaserDistanceMeter::MainRoutine(int count) {
  if (count < 10) return;

  // Body -> Inertial frame
  libra::Vector<3> spacecraft_position_i2b_m = dynamics_.GetOrbit().GetPosition_i_m();
  libra::Quaternion spacecraft_attitude_i2b = dynamics_.GetAttitude().GetQuaternion_i2b().Conjugate();
  libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b);

  // Component -> Inertial frame
  libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

  // Get reflector information
  libra::Vector<3> reflector_position_i_m = inter_spacecraft_communication_.GetCornerCubeReflector().GetReflectorPosition_i_m();
  libra::Vector<3> reflector_normal_direction_i = inter_spacecraft_communication_.GetCornerCubeReflector().GetNormalDirection_i();

  // Conversion
  libra::Vector<3> reflector_position_c_m = dual_quaternion_c2i.InverseTransformVector(reflector_position_i_m);
  libra::Vector<3> reflector_normal_direction_c = dual_quaternion_c2i.GetRotationQuaternion().FrameConversion(reflector_normal_direction_i);  // ?

  // Calc relative distance
  observed_distance_m_ = reflector_position_c_m.CalcNorm();

  // Check reflection
  // Is the reflector in the laser radius?
  double laser_radius_m = observed_distance_m_ * tan(emission_angle_rad_);
  double closest_distance_m = CalcDistanceBwPointAndLine(reflector_position_c_m, libra::Vector<3>{0.0}, laser_emitting_direction_c_);
  if (closest_distance_m > laser_radius_m) {
    is_reflected_ = false;
    return;
  }
  // Is the laser is reflected?
  double reflectable_angle_rad = inter_spacecraft_communication_.GetCornerCubeReflector().GetReflectableAngle_rad();
  double laser_incident_angle_rad = acos(libra::InnerProduct(reflector_normal_direction_c, -laser_emitting_direction_c_));
  if (laser_incident_angle_rad > reflectable_angle_rad) {
    is_reflected_ = false;
    return;
  }
  is_reflected_ = true;

  // Add noise
}

std::string LaserDistanceMeter::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "laser_distance_meter_";
  str_tmp += WriteScalar(head + "is_reflected");
  str_tmp += WriteScalar(head + "observed_distance[m]");

  return str_tmp;
}

std::string LaserDistanceMeter::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(is_reflected_);
  str_tmp += WriteScalar(observed_distance_m_);

  return str_tmp;
}

double LaserDistanceMeter::CalcDistanceBwPointAndLine(libra::Vector<3> point_position, libra::Vector<3> position_on_line,
                                                      libra::Vector<3> line_direction) {
  libra::Vector<3> q_p = point_position - position_on_line;
  double temp = libra::InnerProduct(q_p, line_direction) / pow(line_direction.CalcNorm(), 2.0);
  libra::Vector<3> position = q_p - temp * line_direction;
  return position.CalcNorm();
}
