#include "RelativeOrbitAnalyzer.hpp"

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>

RelativeOrbitAnalyzer::RelativeOrbitAnalyzer(const int prescaler, ClockGenerator* clock_gen, const RelativeInformation& rel_info)
    : ComponentBase(prescaler, clock_gen), rel_info_(rel_info) {
  double target_ra_deg = 10.0;  // TODO: Input parameter
  double target_dec_deg = 0.0;  // TODO: Input parameter
  dcm_eci_to_img_ = MakeDcmEciToImg(target_dec_deg, target_ra_deg);
}

RelativeOrbitAnalyzer::~RelativeOrbitAnalyzer() {}

void RelativeOrbitAnalyzer::MainRoutine(int count) {
  UNUSED(count);
  libra::Vector<3> r_chief_i_m = rel_info_.GetReferenceSatDynamics(0)->GetOrbit().GetSatPosition_i();
  libra::Vector<3> r_target_i_m = rel_info_.GetReferenceSatDynamics(1)->GetOrbit().GetSatPosition_i();

  libra::Vector<3> r_chief_to_target_i_m = rel_info_.GetRelativePosition_i_m(0, 1);
  d_norm_chief_to_target_ = norm(r_chief_to_target_i_m);
  libra::Vector<3> d_chief_to_target_i = normalize(r_chief_to_target_i_m);
  d_chief_to_target_img_ = dcm_eci_to_img_ * d_chief_to_target_i;
}

std::string RelativeOrbitAnalyzer::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeOrbitAnalyzer_";

  str_tmp += WriteVector(head + "baseline_direction", "img", "-", 3);
  str_tmp += WriteScalar(head + "baseline_length", "m");

  return str_tmp;
}

std::string RelativeOrbitAnalyzer::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(d_chief_to_target_img_);
  str_tmp += WriteScalar(d_norm_chief_to_target_);

  return str_tmp;
}

libra::Matrix<3, 3> RelativeOrbitAnalyzer::MakeDcmEciToImg(const double dec_deg, const double ra_deg) {
  const double dec_rad = dec_deg * libra::numbers::deg_to_rad;
  const double ra_rad = ra_deg * libra::numbers::deg_to_rad;
  libra::Vector<3> e_img_z;
  e_img_z[0] = cos(dec_rad) * cos(ra_rad);
  e_img_z[1] = cos(dec_rad) * sin(ra_rad);
  e_img_z[2] = sin(dec_rad);
  libra::Vector<3> e_eci_z;
  e_eci_z[0] = 0.0;
  e_eci_z[1] = 0.0;
  e_eci_z[2] = 1.0;

  libra::Vector<3> e_img_y_tmp, e_img_y;
  e_img_y_tmp = cross(e_eci_z, e_img_z);
  e_img_y_tmp = normalize(e_img_y_tmp);
  e_img_y = cross(e_img_z, e_img_y_tmp);
  e_img_y = normalize(e_img_y);
  libra::Vector<3> e_img_x;
  e_img_x = cross(e_img_y, e_img_z);
  e_img_x = normalize(e_img_x);

  libra::Matrix<3, 3> dcm_eci_to_img;
  for (size_t i = 0; i < 3; i++) {
    dcm_eci_to_img[0][i] = e_img_x[i];
    dcm_eci_to_img[1][i] = e_img_y[i];
    dcm_eci_to_img[2][i] = e_img_z[i];
  }
  return dcm_eci_to_img;
}
