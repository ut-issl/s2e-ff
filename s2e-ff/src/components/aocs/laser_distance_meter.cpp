/**
 * @file laser_distance_meter.hpp
 * @brief Laser distance meter
 */

#include "laser_distance_meter.hpp"

LaserDistanceMeter::LaserDistanceMeter(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics& dynamics,
                                       const FfInterSpacecraftCommunication& inter_spacecraft_communication, const size_t id)
    : Component(prescaler, clock_gen), dynamics_(dynamics), inter_spacecraft_communication_(inter_spacecraft_communication) {
  Initialize(file_name, id);
}

void LaserDistanceMeter::MainRoutine(int count) {
  if (count < 10) return;

  // Body -> Inertial frame
  libra::Vector<3> spacecraft_position_i2b_m = dynamics_.GetOrbit().GetPosition_i_m();
  libra::Quaternion spacecraft_attitude_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
  libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

  // Component -> Inertial frame
  libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

  // Corner cube info
  size_t number_of_reflectors = inter_spacecraft_communication_.GetNumberOfReflectors();
  observed_distance_m_ = 1e30;
  double observed_distance_m = 0.0;
  is_reflected_ = false;
  for (size_t reflector_id = 0; reflector_id < number_of_reflectors; reflector_id++) {
    // Get reflector information
    inter_spacecraft_communication_.GetCornerCubeReflector(reflector_id).Update(count);
    libra::Vector<3> reflector_position_i_m = inter_spacecraft_communication_.GetCornerCubeReflector(reflector_id).GetReflectorPosition_i_m();
    libra::Vector<3> reflector_normal_direction_i = inter_spacecraft_communication_.GetCornerCubeReflector(reflector_id).GetNormalDirection_i();

    // Conversion
    libra::Vector<3> reflector_position_c_m = dual_quaternion_c2i.InverseTransformVector(reflector_position_i_m);
    libra::Vector<3> reflector_normal_direction_c = dual_quaternion_c2i.GetRotationQuaternion().InverseFrameConversion(reflector_normal_direction_i);

    // Calc relative distance
    observed_distance_m = reflector_position_c_m.CalcNorm();

    // Check reflection
    // Is the reflector in the laser radius?
    double laser_radius_m = observed_distance_m * tan(emission_angle_rad_);
    double closest_distance_m = CalcDistanceBwPointAndLine(reflector_position_c_m, libra::Vector<3>{0.0}, laser_emitting_direction_c_);
    if (closest_distance_m > laser_radius_m) {
      continue;
    }
    // Is the laser is reflected?
    double reflectable_angle_rad = inter_spacecraft_communication_.GetCornerCubeReflector(reflector_id).GetReflectableAngle_rad();
    double cos_theta = libra::InnerProduct(reflector_normal_direction_c, -laser_emitting_direction_c_);
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    double laser_incident_angle_rad = acos(cos_theta);
    if (laser_incident_angle_rad > reflectable_angle_rad) {
      continue;
    }
    is_reflected_ = true;
    // Observe closest point
    if (observed_distance_m < observed_distance_m_) {
      observed_distance_m_ = observed_distance_m;
    }
  }

  if (is_reflected_ == true) {
    // Add noise
    // TBW
  } else {
    observed_distance_m_ = 0.0;
  }
}

std::string LaserDistanceMeter::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "laser_distance_meter_" + std::to_string(laser_id_) + "_";
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

// Functions
void LaserDistanceMeter::Initialize(const std::string file_name, const size_t id) {
  IniAccess ini_file(file_name);
  std::string name = "LASER_DISTANCE_METER_";
  const std::string section_name = name + std::to_string(static_cast<long long>(id));
  laser_id_ = id;

  libra::Quaternion quaternion_b2c;
  ini_file.ReadQuaternion(section_name.c_str(), "quaternion_b2c", quaternion_b2c);
  libra::Vector<3> position_b_m;
  ini_file.ReadVector(section_name.c_str(), "position_b_m", position_b_m);
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b_m, quaternion_b2c.Conjugate()).QuaternionConjugate();

  ini_file.ReadVector(section_name.c_str(), "laser_emitting_direction_c", laser_emitting_direction_c_);
  emission_angle_rad_ = ini_file.ReadDouble(section_name.c_str(), "emission_angle_rad");
}
