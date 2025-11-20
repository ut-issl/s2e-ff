/**
 * @file air-drag-control-panel.cpp
 * @brief Class to change satellite air drag panel information
 */

#include "air-drag-control-panel.hpp"

#include <library/math/vector.hpp>
#include <library/math/constants.hpp>

AirDragControlPanel::AirDragControlPanel(ClockGenerator* clock_generator, Surface* surface)
    : Component(1, clock_generator), surface_(surface) {}

AirDragControlPanel::~AirDragControlPanel() {}

void AirDragControlPanel::MainRoutine(const int time_count) {
  UNUSED(time_count);
}

std::string AirDragControlPanel::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string AirDragControlPanel::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
void AirDragControlPanel::SetAngle_deg(double angle_deg) {
  const double angle_rad = angle_deg * libra::deg_to_rad;

  libra::Vector<3> normal_b;
  // Assuming rotation around y-axis
  normal_b[0] = cos(angle_rad);
  normal_b[1] = 0.0;
  normal_b[2] = sin(angle_rad); 
  surface_->SetNormal_b(normal_b);
}
