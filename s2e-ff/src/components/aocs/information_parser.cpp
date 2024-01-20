/**
 * @file information_parser.cpp
 * @brief Information Parser
 */
#include "./information_parser.hpp"

/**
 * @fn InformationParser
 * @brief Constructor
 */
InformationParser::InformationParser(const int prescaler, ClockGenerator* clock_gen, std::vector<QpdPositioningSensor*> qpd_positioning_sensors,
                                     FfInterSpacecraftCommunication& inter_spacecraft_communication)
    : Component(prescaler, clock_gen),
      qpd_positioning_sensors_(qpd_positioning_sensors),
      inter_spacecraft_communication_(inter_spacecraft_communication) {}

void InformationParser::MainRoutine(int count) {
  UNUSED(count);
  double line_of_sight_distance_m = inter_spacecraft_communication_.GetLineOfSightDistance_m();
  for (size_t qpd_id = 0; qpd_id < qpd_positioning_sensors_.size(); ++qpd_id) {
    qpd_positioning_sensors_[qpd_id]->SetErrorCompensatedCoefficient(line_of_sight_distance_m);
    inter_spacecraft_communication_.SetIsLaserReceivedByQpdSensor(qpd_id, qpd_positioning_sensors_[qpd_id]->GetIsReceivedLaser());
    double observed_y_axis_displacement_m = qpd_positioning_sensors_[qpd_id]->GetObservedYAxisDisplacementAfterCompensation_m();
    double observed_z_axis_displacement_m = qpd_positioning_sensors_[qpd_id]->GetObservedZAxisDisplacementAfterCompensation_m();
    inter_spacecraft_communication_.SetDisplacementCalcedByQpdSensor_m(qpd_id, observed_y_axis_displacement_m, observed_z_axis_displacement_m);
  }
}

std::string InformationParser::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string InformationParser::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
