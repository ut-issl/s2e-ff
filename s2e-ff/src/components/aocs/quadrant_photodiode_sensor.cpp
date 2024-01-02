/**
 * @file laser_distance_meter.hpp
 * @brief Laser distance meter
 */

#include "quadrant_photodiode_sensor.hpp"

QuadrantPhotodiodeSensor::QuadrantPhotodiodeSensor(const int prescaler, ClockGenerator* clock_gen, const std::string file_name,
                                                   const Dynamics& dynamics, const FfInterSpacecraftCommunication& inter_spacecraft_communication,
                                                   const size_t id)
    : Component(prescaler, clock_gen), dynamics_(dynamics), inter_spacecraft_communication_(inter_spacecraft_communication) {
  Initialize(file_name, id);
}

void QuadrantPhotodiodeSensor::MainRoutine(int count) {
  if (count < 10) return;

  // Body -> Inertial frame
  libra::Vector<3> spacecraft_position_i2b_m = dynamics_.GetOrbit().GetPosition_i_m();
  libra::Quaternion spacecraft_attitude_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
  libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

  // Component -> Inertial frame
  libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

  // Laser emitter info
  size_t number_of_laser_emitters = inter_spacecraft_communication_.GetNumberOfLasers();
  actual_distance_m_ = 1e30;
  actual_horizontal_displacement_m_ = 1e30;
  actual_vertical_displacement_m_ = 1e30;
  observed_horizontal_displacement_m_ = 0.5;
  observed_vertical_displacement_m_ = 0.5;
  for (size_t qpd_sensor_output_id = 0; qpd_sensor_output_id < 3; qpd_sensor_output_id++) {
    qpd_sensor_output_V_[qpd_sensor_output_id] = 0.0;
  }
  double qpd_laser_distance_m = 0.0;
  double qpd_horizontal_displacement_m = 0.0;
  double qpd_vertical_displacement_m = 0.0;

  double qpd_received_laser_beam_radius_m = 0.0;
  double qpd_received_laser_power_W = 0.0;

  is_received_laser_ = false;
  for (size_t laser_id = 0; laser_id < number_of_laser_emitters; laser_id++) {
    // Get laser information
    libra::Vector<3> laser_position_i_m = inter_spacecraft_communication_.GetLaserEmitter(laser_id).GetLaserPosition_i_m();
    libra::Vector<3> laser_emitting_direction_i = inter_spacecraft_communication_.GetLaserEmitter(laser_id).GetEmittingDirection_i();

    // Conversion
    libra::Vector<3> laser_position_c_m = dual_quaternion_c2i.InverseTransformVector(laser_position_i_m);
    libra::Vector<3> laser_emitting_direction_c = dual_quaternion_c2i.GetRotationQuaternion().InverseFrameConversion(laser_emitting_direction_i);

    double cos_theta = libra::InnerProduct(qpd_normal_direction_c_, -laser_emitting_direction_c);
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    double qpd_laser_received_angle_rad = cos_theta;

    // Calc relative position displacement (horizontal direction and vertical direction)
    if (acos(qpd_laser_received_angle_rad) > qpd_laser_received_angle_rad_) {
      continue;
    }
    libra::Vector<3> laser_received_position_c_m =
        CalcLaserReceivedPosition(laser_position_c_m, libra::Vector<3>{0.0}, qpd_normal_direction_c_, laser_emitting_direction_c);
    qpd_laser_distance_m = laser_position_c_m.CalcNorm();
    qpd_horizontal_displacement_m = CalcDisplacement(laser_received_position_c_m, libra::Vector<3>{0.0}, qpd_horizontal_direction_c_);
    qpd_vertical_displacement_m = CalcDisplacement(laser_received_position_c_m, libra::Vector<3>{0.0}, qpd_vertical_direction_c_);

    if (qpd_laser_distance_m < actual_distance_m_) {
      actual_distance_m_ = qpd_laser_distance_m;
    }
    if (fabs(qpd_horizontal_displacement_m) < fabs(actual_horizontal_displacement_m_)) {
      actual_horizontal_displacement_m_ = qpd_horizontal_displacement_m;
    }
    if (fabs(qpd_vertical_displacement_m) < fabs(actual_vertical_displacement_m_)) {
      actual_vertical_displacement_m_ = qpd_vertical_displacement_m;
    }

    qpd_received_laser_beam_radius_m = inter_spacecraft_communication_.GetLaserEmitter(laser_id).GetBeamRadius_m(qpd_laser_distance_m);
    qpd_received_laser_power_W = inter_spacecraft_communication_.GetLaserEmitter(laser_id).GetEmissionPower_W();

    CalcSensorOutput(qpd_received_laser_power_W, qpd_received_laser_beam_radius_m, qpd_horizontal_displacement_m, qpd_vertical_displacement_m);

    if (pow(fabs(qpd_horizontal_displacement_m * qpd_vertical_displacement_m), 0.5) > 2 * qpd_sensor_radius_m_) {
      continue;
    }
    is_received_laser_ = true;

    // FIXME: Need to implement the correct algorithm
    observed_horizontal_displacement_m_ = actual_horizontal_displacement_m_;
    observed_vertical_displacement_m_ = actual_vertical_displacement_m_;
  }
}

std::string QuadrantPhotodiodeSensor::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "quadrant_photodiode_sensor_";
  str_tmp += WriteScalar(head + "is_received_laser");
  str_tmp += WriteScalar(head + "actual_distance[m]");
  str_tmp += WriteScalar(head + "actual_horizontal_displacement[m]");
  str_tmp += WriteScalar(head + "actual_vertical_displacement[m]");

  return str_tmp;
}

std::string QuadrantPhotodiodeSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(is_received_laser_);
  str_tmp += WriteScalar(actual_distance_m_);
  str_tmp += WriteScalar(actual_horizontal_displacement_m_);
  str_tmp += WriteScalar(actual_vertical_displacement_m_);

  return str_tmp;
}

double QuadrantPhotodiodeSensor::CalcDistanceBwPointAndLine(libra::Vector<3> point_position, libra::Vector<3> position_on_line,
                                                            libra::Vector<3> line_direction) {
  libra::Vector<3> q_p = point_position - position_on_line;
  double temp = libra::InnerProduct(q_p, line_direction) / pow(line_direction.CalcNorm(), 2.0);
  libra::Vector<3> position = q_p - temp * line_direction;
  return position.CalcNorm();
}

libra::Vector<3> QuadrantPhotodiodeSensor::CalcLaserReceivedPosition(libra::Vector<3> point_position, libra::Vector<3> origin_position,
                                                                     libra::Vector<3> plane_normal_direction, libra::Vector<3> point_line_direction) {
  libra::Vector<3> q_p = point_position - origin_position;
  double temp1 = libra::InnerProduct(q_p, plane_normal_direction) / pow(plane_normal_direction.CalcNorm(), 2.0);
  libra::Vector<3> position_temp1 = q_p - temp1 * plane_normal_direction;
  double temp2 = temp1 * pow(plane_normal_direction.CalcNorm(), 2.0) / libra::InnerProduct(plane_normal_direction, point_line_direction);
  libra::Vector<3> position_temp2 = temp2 * point_line_direction - temp1 * plane_normal_direction;
  libra::Vector<3> position = position_temp1 + position_temp2;
  return position;
}

double QuadrantPhotodiodeSensor::CalcDisplacement(libra::Vector<3> point_position, libra::Vector<3> origin_position,
                                                  libra::Vector<3> displacement_direction) {
  libra::Vector<3> q_p = point_position - origin_position;
  double displacement_m = libra::InnerProduct(q_p, displacement_direction) / pow(displacement_direction.CalcNorm(), 2.0);
  return displacement_m;
};

void QuadrantPhotodiodeSensor::CalcSensorOutput(double laser_power_W, double laser_beam_radius_m, double qpd_horizontal_displacement_m,
                                                double qpd_vertical_displacement_m) {
  double integral_step_m = 5.0e-5;
  for (size_t horizontal_step = 0; horizontal_step <= (size_t)(qpd_sensor_radius_m_ / integral_step_m) * 2; horizontal_step++) {
    double horizontal_pos_m = integral_step_m * horizontal_step - qpd_sensor_radius_m_;
    double vertical_range_max_m =
        (double)((int32_t)(pow(pow(qpd_sensor_radius_m_, 2.0) - pow(horizontal_pos_m, 2.0), 0.5) / integral_step_m) * integral_step_m);
    for (size_t vertical_step = 0; vertical_step <= (size_t)(vertical_range_max_m / integral_step_m) * 2; vertical_step++) {
      double vertical_pos_m = integral_step_m * vertical_step - vertical_range_max_m;
      double temp = 2 * laser_power_W / libra::pi / pow(laser_beam_radius_m, 2.0) *
                    exp(-2 * (pow((horizontal_pos_m - qpd_horizontal_displacement_m) / laser_beam_radius_m, 2.0) +
                              pow((vertical_pos_m - qpd_vertical_displacement_m) / laser_beam_radius_m, 2.0))) *
                    pow(integral_step_m, 2.0);
      qpd_sensor_output_V_[0] += sgn(-horizontal_pos_m) * temp;
      qpd_sensor_output_V_[1] += sgn(vertical_pos_m) * temp;
      qpd_sensor_output_V_[2] += temp;
    }
  }
}

// Functions
void QuadrantPhotodiodeSensor::Initialize(const std::string file_name, const size_t id) {
  IniAccess ini_file(file_name);
  std::string name = "QUADRANT_PHOTODIODE_SENSOR_";
  const std::string section_name = name + std::to_string(static_cast<long long>(id));

  libra::Quaternion quaternion_b2c;
  ini_file.ReadQuaternion(section_name.c_str(), "quaternion_b2c", quaternion_b2c);
  libra::Vector<3> position_b_m;
  ini_file.ReadVector(section_name.c_str(), "position_b_m", position_b_m);
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b_m, quaternion_b2c.Conjugate()).QuaternionConjugate();

  ini_file.ReadVector(section_name.c_str(), "qpd_horizontal_direction_c", qpd_horizontal_direction_c_);
  ini_file.ReadVector(section_name.c_str(), "qpd_vertical_direction_c", qpd_vertical_direction_c_);
  qpd_normal_direction_c_ = libra::OuterProduct(qpd_horizontal_direction_c_, qpd_vertical_direction_c_);
  qpd_sensor_radius_m_ = ini_file.ReadDouble(section_name.c_str(), "qpd_sensor_radius_m");
  qpd_laser_received_angle_rad_ = ini_file.ReadDouble(section_name.c_str(), "qpd_laser_received_angle_rad");
}
