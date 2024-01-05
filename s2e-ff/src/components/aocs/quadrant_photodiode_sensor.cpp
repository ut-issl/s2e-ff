/**
 * @file quadrant_photodiode_sensor.cpp
 * @brief Quadrant photodiode (QPD) sensor
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
  distance_true_m_ = 1e30;
  y_axis_displacement_true_m_ = 1e30;
  z_axis_displacement_true_m_ = 1e30;
  observed_y_axis_displacement_m_ = 1.0e3;
  observed_z_axis_displacement_m_ = 1.0e3;
  qpd_sensor_output_y_axis_V_ = 0.0;
  qpd_sensor_output_z_axis_V_ = 0.0;
  qpd_sensor_output_sum_V_ = 0.0;
  double qpd_laser_distance_m = 0.0;
  double qpd_y_axis_displacement_m = 0.0;
  double qpd_z_axis_displacement_m = 0.0;

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
    double qpd_laser_received_angle_rad = acos(cos_theta);

    // Calc relative position displacement (horizontal direction and vertical direction)
    if (qpd_laser_received_angle_rad > qpd_laser_receivable_angle_rad_) {
      continue;
    }
    libra::Vector<3> laser_received_position_c_m =
        CalcLaserReceivedPosition(laser_position_c_m, libra::Vector<3>{0.0}, qpd_normal_direction_c_, laser_emitting_direction_c);
    qpd_laser_distance_m = laser_position_c_m.CalcNorm();
    qpd_y_axis_displacement_m = CalcDisplacement(laser_received_position_c_m, libra::Vector<3>{0.0}, qpd_horizontal_direction_c_);
    qpd_z_axis_displacement_m = CalcDisplacement(laser_received_position_c_m, libra::Vector<3>{0.0}, qpd_vertical_direction_c_);

    if (qpd_laser_distance_m < distance_true_m_) {
      distance_true_m_ = qpd_laser_distance_m;
    }
    if (fabs(qpd_y_axis_displacement_m) < fabs(y_axis_displacement_true_m_)) {
      y_axis_displacement_true_m_ = qpd_y_axis_displacement_m;
    }
    if (fabs(qpd_z_axis_displacement_m) < fabs(z_axis_displacement_true_m_)) {
      z_axis_displacement_true_m_ = qpd_z_axis_displacement_m;
    }

    qpd_received_laser_beam_radius_m = inter_spacecraft_communication_.GetLaserEmitter(laser_id).GetBeamWidthRadius_m(qpd_laser_distance_m);
    qpd_received_laser_power_W = inter_spacecraft_communication_.GetLaserEmitter(laser_id).GetEmissionPower_W();

    CalcSensorOutput(qpd_received_laser_power_W, qpd_received_laser_beam_radius_m, qpd_y_axis_displacement_m, qpd_z_axis_displacement_m);

    if (qpd_sensor_output_sum_V_ < qpd_sensor_output_voltage_threshold_V_) {
      continue;
    }
    is_received_laser_ = true;
  }
  if (is_received_laser_) {
    observed_y_axis_displacement_m_ = DeterminePositionDisplacement(yAxisDirection);
    observed_z_axis_displacement_m_ = DeterminePositionDisplacement(zAxisDirection);
  }
}

std::string QuadrantPhotodiodeSensor::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "quadrant_photodiode_sensor_";
  str_tmp += WriteScalar(head + "is_received_laser");
  str_tmp += WriteScalar(head + "distance_true[m]");
  str_tmp += WriteScalar(head + "y_axis_displacement_true[m]");
  str_tmp += WriteScalar(head + "z_axis_displacement_true[m]");
  str_tmp += WriteScalar(head + "qpd_output_value_y_axis[V]");
  str_tmp += WriteScalar(head + "qpd_output_value_z_axis[V]");
  str_tmp += WriteScalar(head + "qpd_output_value_sum[V]");
  str_tmp += WriteScalar(head + "observed_y_axis_displacement[m]");
  str_tmp += WriteScalar(head + "observed_z_axis_displacement[m]");

  return str_tmp;
}

std::string QuadrantPhotodiodeSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(is_received_laser_);
  str_tmp += WriteScalar(distance_true_m_);
  str_tmp += WriteScalar(y_axis_displacement_true_m_);
  str_tmp += WriteScalar(z_axis_displacement_true_m_);
  str_tmp += WriteScalar(qpd_sensor_output_y_axis_V_);
  str_tmp += WriteScalar(qpd_sensor_output_z_axis_V_);
  str_tmp += WriteScalar(qpd_sensor_output_sum_V_);
  str_tmp += WriteScalar(observed_y_axis_displacement_m_);
  str_tmp += WriteScalar(observed_z_axis_displacement_m_);

  return str_tmp;
}

libra::Vector<3> QuadrantPhotodiodeSensor::CalcLaserReceivedPosition(const libra::Vector<3> point_position, const libra::Vector<3> origin_position,
                                                                     const libra::Vector<3> plane_normal_direction,
                                                                     const libra::Vector<3> point_line_direction) {
  libra::Vector<3> q_p = point_position - origin_position;
  double temp1 = libra::InnerProduct(q_p, plane_normal_direction) / pow(plane_normal_direction.CalcNorm(), 2.0);
  libra::Vector<3> position_temp1 = q_p - temp1 * plane_normal_direction;
  double temp2 = temp1 * pow(plane_normal_direction.CalcNorm(), 2.0) / libra::InnerProduct(plane_normal_direction, point_line_direction);
  libra::Vector<3> position_temp2 = temp2 * point_line_direction - temp1 * plane_normal_direction;
  libra::Vector<3> position = position_temp1 + position_temp2;
  return position;
}

double QuadrantPhotodiodeSensor::CalcDisplacement(const libra::Vector<3> point_position, const libra::Vector<3> origin_position,
                                                  const libra::Vector<3> displacement_direction) {
  libra::Vector<3> q_p = point_position - origin_position;
  double displacement_m = libra::InnerProduct(q_p, displacement_direction) / pow(displacement_direction.CalcNorm(), 2.0);
  return displacement_m;
};

void QuadrantPhotodiodeSensor::CalcSensorOutput(const double laser_power_W, const double laser_beam_radius_m, const double qpd_y_axis_displacement_m,
                                                const double qpd_z_axis_displacement_m) {
  qpd_sensor_radius_m_ = (double)(((int32_t)(qpd_sensor_radius_m_ / qpd_sensor_integral_step_m_)) * qpd_sensor_integral_step_m_);
  for (size_t horizontal_step = 0; horizontal_step <= (size_t)(qpd_sensor_radius_m_ / qpd_sensor_integral_step_m_) * 2; horizontal_step++) {
    double horizontal_pos_m = qpd_sensor_integral_step_m_ * horizontal_step - qpd_sensor_radius_m_;
    double vertical_range_max_m =
        (double)((int32_t)(sqrt(pow(qpd_sensor_radius_m_, 2.0) - pow(horizontal_pos_m, 2.0)) / qpd_sensor_integral_step_m_) *
                 qpd_sensor_integral_step_m_);
    for (size_t vertical_step = 0; vertical_step <= (size_t)(vertical_range_max_m / qpd_sensor_integral_step_m_) * 2; vertical_step++) {
      double vertical_pos_m = qpd_sensor_integral_step_m_ * vertical_step - vertical_range_max_m;
      double temp = 2 * laser_power_W / libra::pi / pow(laser_beam_radius_m, 2.0) *
                    exp(-2 * (pow((horizontal_pos_m - qpd_y_axis_displacement_m) / laser_beam_radius_m, 2.0) +
                              pow((vertical_pos_m - qpd_z_axis_displacement_m) / laser_beam_radius_m, 2.0))) *
                    pow(qpd_sensor_integral_step_m_, 2.0);
      qpd_sensor_output_y_axis_V_ += CalcSign(-horizontal_pos_m, qpd_sensor_integral_step_m_ / 2) * temp;
      qpd_sensor_output_z_axis_V_ += CalcSign(vertical_pos_m, qpd_sensor_integral_step_m_ / 2) * temp;
      qpd_sensor_output_sum_V_ += temp;
    }
  }
}

double QuadrantPhotodiodeSensor::CalcSign(const double input_value, const double threshold) {
  if (input_value < -threshold) {
    return -1;
  } else if (input_value > threshold) {
    return 1;
  }
  return 0.0;
}

double QuadrantPhotodiodeSensor::DeterminePositionDisplacement(const QpdPositionDeterminationDirection determination_direction) {
  double determined_displacement_m = 0.0;
  double sensor_value_ratio;
  switch (determination_direction) {
    case yAxisDirection:
      determined_displacement_m = -CalcSign(qpd_sensor_output_y_axis_V_, 0.0) * qpd_sensor_position_determination_threshold_m_;
      sensor_value_ratio = qpd_sensor_output_y_axis_V_ / qpd_sensor_output_sum_V_;
      for (size_t i = 0; i < qpd_ratio_y_reference_list_.size() - 1; i++) {
        if (sensor_value_ratio <= qpd_ratio_y_reference_list_[i] && sensor_value_ratio >= qpd_ratio_y_reference_list_[i + 1]) {
          determined_displacement_m = qpd_displacement_reference_list_m_[i];
          determined_displacement_m += (qpd_displacement_reference_list_m_[i + 1] - qpd_displacement_reference_list_m_[i]) *
                                       (sensor_value_ratio - qpd_ratio_y_reference_list_[i]) /
                                       (qpd_ratio_y_reference_list_[i + 1] - qpd_ratio_y_reference_list_[i]);
        }
      }
      break;
    case zAxisDirection:
      determined_displacement_m = CalcSign(qpd_sensor_output_z_axis_V_, 0.0) * qpd_sensor_position_determination_threshold_m_;
      sensor_value_ratio = qpd_sensor_output_z_axis_V_ / qpd_sensor_output_sum_V_;
      for (size_t i = 0; i < qpd_ratio_y_reference_list_.size() - 1; i++) {
        if (sensor_value_ratio >= qpd_ratio_z_reference_list_[i] && sensor_value_ratio <= qpd_ratio_z_reference_list_[i + 1]) {
          determined_displacement_m = qpd_displacement_reference_list_m_[i];
          determined_displacement_m += (qpd_displacement_reference_list_m_[i + 1] - qpd_displacement_reference_list_m_[i]) *
                                       (sensor_value_ratio - qpd_ratio_z_reference_list_[i]) /
                                       (qpd_ratio_z_reference_list_[i + 1] - qpd_ratio_z_reference_list_[i]);
        }
      }
      break;
    default:
      // NOT REACHED
      break;
  }
  return determined_displacement_m;
}

// Functions
void QuadrantPhotodiodeSensor::Initialize(const std::string file_name, const size_t id) {
  IniAccess ini_file(file_name);
  std::string name = "QUADRANT_PHOTODIODE_SENSOR_";
  const std::string section_name = name + std::to_string(static_cast<long long>(id));

  std::string file_path = ini_file.ReadString(section_name.c_str(), "qpd_sensor_file_directory");
  std::string filepath_qpd_sensor_reference = file_path + "qpd_sensor_reference.csv";
  IniAccess conf_qpd_sensor_reference(filepath_qpd_sensor_reference);
  std::vector<std::vector<std::string>> qpd_sensor_reference_str_list;
  conf_qpd_sensor_reference.ReadCsvString(qpd_sensor_reference_str_list, 1000);

  for (size_t index = 1; index < qpd_sensor_reference_str_list.size(); ++index) {  // first row is for labels
    qpd_displacement_reference_list_m_.push_back(stod(qpd_sensor_reference_str_list[index][0]));
    qpd_ratio_y_reference_list_.push_back(stod(qpd_sensor_reference_str_list[index][1]));
    qpd_ratio_z_reference_list_.push_back(stod(qpd_sensor_reference_str_list[index][2]));
  }

  libra::Quaternion quaternion_b2c;
  ini_file.ReadQuaternion(section_name.c_str(), "quaternion_b2c", quaternion_b2c);
  libra::Vector<3> position_b_m;
  ini_file.ReadVector(section_name.c_str(), "position_b_m", position_b_m);
  dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b_m, quaternion_b2c.Conjugate()).QuaternionConjugate();

  ini_file.ReadVector(section_name.c_str(), "qpd_horizontal_direction_c", qpd_horizontal_direction_c_);
  ini_file.ReadVector(section_name.c_str(), "qpd_vertical_direction_c", qpd_vertical_direction_c_);
  qpd_normal_direction_c_ = libra::OuterProduct(qpd_horizontal_direction_c_, qpd_vertical_direction_c_);
  qpd_sensor_radius_m_ = ini_file.ReadDouble(section_name.c_str(), "qpd_sensor_radius_m");
  qpd_sensor_integral_step_m_ = ini_file.ReadDouble(section_name.c_str(), "qpd_sensor_integral_step_m");
  qpd_sensor_position_determination_threshold_m_ = ini_file.ReadDouble(section_name.c_str(), "qpd_sensor_position_determination_threshold_m");
  qpd_laser_receivable_angle_rad_ = ini_file.ReadDouble(section_name.c_str(), "qpd_laser_receivable_angle_rad");
  qpd_sensor_output_voltage_threshold_V_ = ini_file.ReadDouble(section_name.c_str(), "qpd_sensor_output_voltage_threshold_V");
}
