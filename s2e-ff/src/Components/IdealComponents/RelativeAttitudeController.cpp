#include "RelativeAttitudeController.hpp"

#include <Library/math/Matrix.hpp>

#define THRESHOLD_CA cos(30.0 / 180.0 * libra::pi)  // fix me

// Constructor
RelativeAttitudeController::RelativeAttitudeController(const int prescaler, ClockGenerator* clock_gen, const RelativeInformation& rel_info,
                                                       const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics)
    : ComponentBase(prescaler, clock_gen), rel_info_(rel_info), local_celes_info_(local_celes_info), dynamics_(dynamics) {
  target_sat_id_ = 1;
  my_sat_id_ = 0;

  main_mode_ = RelativeAttitudeControlMode::TARGET_SATELLITE_POINTING;
  pointing_main_target_b_[0] = 1.0;
  pointing_main_target_b_[1] = 0.0;
  pointing_main_target_b_[2] = 0.0;

  sub_mode_ = RelativeAttitudeControlMode::ORBIT_NORMAL_POINTING;
  pointing_sub_target_b_[0] = 0.0;
  pointing_sub_target_b_[1] = 1.0;
  pointing_sub_target_b_[2] = 0.0;

  is_calc_enabled_ = true;
}

RelativeAttitudeController::~RelativeAttitudeController() {}

void RelativeAttitudeController::MainRoutine(int count) {
  UNUSED(count);
  if (!is_calc_enabled_) return;

  libra::Vector<3> main_direction_i, sub_direction_i;
  // Calc main target direction
  main_direction_i = CalcTargetDirection_i(main_mode_);
  // Calc sub target direction
  sub_direction_i = CalcTargetDirection_i(sub_mode_);

  // Calc attitude
  libra::Quaternion q_i2b = CalcTargetQuaternion(main_direction_i, sub_direction_i);
  dynamics_.SetAttitude().SetQuaternion_i2b(q_i2b);
}

void RelativeAttitudeController::PowerOffRoutine() {}

std::string RelativeAttitudeController::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "RelativeAttitudeController_";
  str_tmp += WriteScalar(head + "main_mode", "-");

  return str_tmp;
}

std::string RelativeAttitudeController::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar((int)main_mode_);

  return str_tmp;
}

// Internal functions
void RelativeAttitudeController::Initialize(void) {
  if (main_mode_ >= RelativeAttitudeControlMode::NO_CONTROL) is_calc_enabled_ = false;
  if (sub_mode_ >= RelativeAttitudeControlMode::NO_CONTROL) is_calc_enabled_ = false;

  // sub mode check
  if (main_mode_ == sub_mode_) {
    std::cout << "sub mode should not equal to main mode. \n";
    is_calc_enabled_ = false;
    return;
  }
  // pointing direction check
  normalize(pointing_main_target_b_);
  normalize(pointing_sub_target_b_);
  double tmp = inner_product(pointing_main_target_b_, pointing_sub_target_b_);
  tmp = std::abs(tmp);
  if (tmp > THRESHOLD_CA) {
    std::cout << "sub target direction should separate from the main target direction. \n";
    is_calc_enabled_ = false;
    return;
  }

  return;
}

libra::Vector<3> RelativeAttitudeController::CalcTargetDirection_i(RelativeAttitudeControlMode mode) {
  libra::Vector<3> direction_i;
  switch (mode) {
    case RelativeAttitudeControlMode::TARGET_SATELLITE_POINTING:
      direction_i = rel_info_.GetRelativePosition_i_m(target_sat_id_, my_sat_id_);
      break;
    case RelativeAttitudeControlMode::ORBIT_NORMAL_POINTING:
      direction_i = outer_product(dynamics_.GetOrbit().GetSatPosition_i(), dynamics_.GetOrbit().GetSatVelocity_i());
      break;
    default:
      // Not Reached
      break;
  }

  normalize(direction_i);
  return direction_i;
}

libra::Quaternion RelativeAttitudeController::CalcTargetQuaternion(const libra::Vector<3> main_direction_i, const libra::Vector<3> sub_direction_i) {
  // Calc DCM ECI->Target
  libra::Matrix<3, 3> DCM_t2i = CalcDcmFromVectors(main_direction_i, sub_direction_i);
  // Calc DCM Target->body
  libra::Matrix<3, 3> DCM_t2b = CalcDcmFromVectors(pointing_main_target_b_, pointing_sub_target_b_);
  // Calc DCM ECI->body
  libra::Matrix<3, 3> DCM_i2b = DCM_t2b * transpose(DCM_t2i);
  // Convert to Quaternion
  return libra::Quaternion::fromDCM(DCM_i2b);
}

libra::Matrix<3, 3> RelativeAttitudeController::CalcDcmFromVectors(const libra::Vector<3> main_direction, const libra::Vector<3> sub_direction) {
  libra::Matrix<3, 3> dcm = libra::eye<3>();

  // Check vector
  double tmp = inner_product(main_direction, sub_direction);
  tmp = std::abs(tmp);
  if (tmp > 0.99999) {
    return dcm;
  }

  // Calc basis vectors
  libra::Vector<3> ex, ey, ez;
  ex = main_direction;
  libra::Vector<3> tmp1 = outer_product(ex, sub_direction);
  libra::Vector<3> tmp2 = outer_product(tmp1, ex);
  ey = normalize(tmp2);
  libra::Vector<3> tmp3 = outer_product(ex, ey);
  ez = normalize(tmp3);

  // Generate DCM
  for (int i = 0; i < 3; i++) {
    dcm[i][0] = ex[i];
    dcm[i][1] = ey[i];
    dcm[i][2] = ez[i];
  }
  return dcm;
}
