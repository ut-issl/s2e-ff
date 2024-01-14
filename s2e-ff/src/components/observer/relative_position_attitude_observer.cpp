/**
 * @file relative_position_attitude_observer.cpp
 * @brief Relative Position and Attitude Observer
 */

#include "./relative_position_attitude_observer.hpp"

RelativePositionAttitudeObserver::RelativePositionAttitudeObserver(const int prescaler, ClockGenerator* clock_gen,
                                                                   std::vector<LaserDistanceMeter*>& laser_distance_meters,
                                                                   std::vector<QpdPositioningSensor*>& qpd_positioning_sensors)
    : Component(prescaler, clock_gen), laser_distance_meters_(laser_distance_meters), qpd_positioning_sensors_(qpd_positioning_sensors) {}

void RelativePositionAttitudeObserver::MainRoutine(int count) {
  UNUSED(count);
  ObserveRelativePositionAttitude();
}

std::string RelativePositionAttitudeObserver::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "relative_position_attitude_observer_";
  str_tmp += WriteVector(head + "position", "tb2rb", "m", 3);
  str_tmp += WriteVector(head + "eular_angle", "tb2rb", "rad", 3);
  return str_tmp;
}

std::string RelativePositionAttitudeObserver::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(observed_relative_position_m_);
  str_tmp += WriteVector(observed_relative_euler_angle_rad_);
  return str_tmp;
}

// Functions
void RelativePositionAttitudeObserver::ObserveRelativePositionAttitude() {
  libra::Vector<kLaserDistanceMetersNumber> line_of_sight_distance_m{0.0};

  for (size_t laser_id = 0; laser_id < kLaserDistanceMetersNumber; ++laser_id) {
    line_of_sight_distance_m[laser_id] = -laser_distance_meters_[laser_id]->GetObservedDistance_m();
  }

  observed_relative_position_m_[0] = (line_of_sight_distance_m[0] + line_of_sight_distance_m[2]) / 2.0;
  observed_relative_euler_angle_rad_[1] = (line_of_sight_distance_m[1] - line_of_sight_distance_m[2]) / 2.0 / component_position_z_axis_m_;
  observed_relative_euler_angle_rad_[2] = (line_of_sight_distance_m[1] - line_of_sight_distance_m[0]) / 2.0 / component_position_y_axis_m_;

  libra::Vector<kQpdPositioningSensorsNumber> displacement_y_axis_m{0.0};
  libra::Vector<kQpdPositioningSensorsNumber> displacement_z_axis_m{0.0};
  for (size_t qpd_id = 0; qpd_id < kQpdPositioningSensorsNumber; ++qpd_id) {
    displacement_y_axis_m[qpd_id] =
        qpd_positioning_sensors_[qpd_id]->GetObservedYAxisDisplacementAfterCompensation_m(observed_relative_position_m_[0]);
    displacement_z_axis_m[qpd_id] =
        qpd_positioning_sensors_[qpd_id]->GetObservedZAxisDisplacementAfterCompensation_m(observed_relative_position_m_[0]);
  }
  observed_relative_position_m_[1] = (displacement_y_axis_m[0] + displacement_y_axis_m[1]) / 2.0;
  observed_relative_position_m_[2] = (displacement_z_axis_m[0] + displacement_z_axis_m[1]) / 2.0;
  observed_relative_euler_angle_rad_[0] = (displacement_y_axis_m[1] + displacement_y_axis_m[0]) / 2.0 / component_position_z_axis_m_;
}
