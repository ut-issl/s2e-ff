#include "relative_orbit_analyzer.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/math/constants.hpp>

RelativeOrbitAnalyzer::RelativeOrbitAnalyzer(const int prescaler, ClockGenerator* clock_gen, const RelativeInformation& rel_info)
    : Component(prescaler, clock_gen), rel_info_(rel_info) {
  double target_ra_deg = 0.0;    // TODO: Input parameter
  double target_dec_deg = 10.0;  // TODO: Input parameter
  dcm_eci_to_img_ = MakeDcmEciToImg(target_dec_deg, target_ra_deg);
}

RelativeOrbitAnalyzer::~RelativeOrbitAnalyzer() {}

void RelativeOrbitAnalyzer::MainRoutine(int count) {
  UNUSED(count);

  // Target-1
  libra::Vector<3> r_chief_to_target1_i_m = rel_info_.GetRelativePosition_i_m(1, 0);
  d_norm_chief_to_target1_ = r_chief_to_target1_i_m.CalcNorm();
  libra::Vector<3> d_chief_to_target1_i = r_chief_to_target1_i_m.CalcNormalizedVector();
  d_chief_to_target1_img_ = dcm_eci_to_img_ * d_chief_to_target1_i;

  baseline_angle1_in_img_rad_ = asin(d_chief_to_target1_img_[2]);

  // Target-2
  libra::Vector<3> r_chief_to_target2_i_m = rel_info_.GetRelativePosition_i_m(2, 0);
  d_norm_chief_to_target2_ = r_chief_to_target2_i_m.CalcNorm();
  libra::Vector<3> d_chief_to_target2_i = r_chief_to_target2_i_m.CalcNormalizedVector();
  d_chief_to_target2_img_ = dcm_eci_to_img_ * d_chief_to_target2_i;

  baseline_angle2_in_img_rad_ = asin(d_chief_to_target2_img_[2]);
}

std::string RelativeOrbitAnalyzer::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "RelativeOrbitAnalyzer_";

  str_tmp += WriteVector(head + "baseline_direction_target1", "img", "-", 3);
  str_tmp += WriteScalar(head + "intersat_length_target1", "m");
  str_tmp += WriteScalar(head + "baseline_angle_in_img_target1", "rad");

  str_tmp += WriteVector(head + "baseline_direction_target2", "img", "-", 3);
  str_tmp += WriteScalar(head + "intersat_length_target2", "m");
  str_tmp += WriteScalar(head + "baseline_angle_in_img_target2", "rad");

  return str_tmp;
}

std::string RelativeOrbitAnalyzer::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(d_chief_to_target1_img_);
  str_tmp += WriteScalar(d_norm_chief_to_target1_);
  str_tmp += WriteScalar(baseline_angle1_in_img_rad_);

  str_tmp += WriteVector(d_chief_to_target2_img_);
  str_tmp += WriteScalar(d_norm_chief_to_target2_);
  str_tmp += WriteScalar(baseline_angle2_in_img_rad_);

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
  e_img_y_tmp = e_img_y_tmp.CalcNormalizedVector();
  e_img_y = cross(e_img_z, e_img_y_tmp);
  e_img_y = e_img_y.CalcNormalizedVector();
  libra::Vector<3> e_img_x;
  e_img_x = cross(e_img_y, e_img_z);
  e_img_x = e_img_x.CalcNormalizedVector();

  libra::Matrix<3, 3> dcm_eci_to_img;
  for (size_t i = 0; i < 3; i++) {
    dcm_eci_to_img[0][i] = e_img_x[i];
    dcm_eci_to_img[1][i] = e_img_y[i];
    dcm_eci_to_img[2][i] = e_img_z[i];
  }
  return dcm_eci_to_img;
}
