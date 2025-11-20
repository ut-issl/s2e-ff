/**
 * @file air-drag-control-panel.cpp
 * @brief Class to change satellite air drag panel information
 */

#include "air-drag-control-panel.hpp"

#include <library/math/vector.hpp>
#include <library/math/constants.hpp>

AirDragControlPanel::AirDragControlPanel(ClockGenerator* clock_generator, Surface* surface)
    : Component(1, clock_generator), surface_(surface) {
  area_m2_ = surface_->GetArea_m2();
  angle_deg_ = 0.0;
}

AirDragControlPanel::~AirDragControlPanel() {}

void AirDragControlPanel::MainRoutine(const int time_count) {
  UNUSED(time_count);
  surface_->SetArea_m2(area_m2_);

  libra::Vector<3> normal_b;
  // Assuming rotation around y-axis
  const double angle_rad = angle_deg_ * libra::deg_to_rad;

  normal_b[0] = cos(angle_rad);
  normal_b[1] = 0.0;
  normal_b[2] = sin(angle_rad); 
  surface_->SetNormal_b(normal_b);

}

std::string AirDragControlPanel::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "AirDragPanel_";
  str_tmp += WriteScalar(head + "area", "m2");
  str_tmp += WriteScalar(head + "angle", "deg");

  return str_tmp;
}

std::string AirDragControlPanel::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(area_m2_);
  str_tmp += WriteScalar(angle_deg_);

  return str_tmp;
}
