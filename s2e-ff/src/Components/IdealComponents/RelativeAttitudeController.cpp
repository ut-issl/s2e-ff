#include "RelativeAttitudeController.hpp"

#include <library/math/matrix.hpp>

#define THRESHOLD_CA cos(30.0 / 180.0 * libra::pi)  // fix me

// Constructor
RelativeAttitudeController::RelativeAttitudeController(const int prescaler, ClockGenerator* clock_gen, const RelativeAttitudeControlMode main_mode,
                                                       const RelativeAttitudeControlMode sub_mode, const libra::Vector<3> main_target_direction_b,
                                                       const libra::Vector<3> sub_target_direction_b, const int target_sat_id,
                                                       const int reference_sat_id, const RelativeInformation& rel_info,
                                                       const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics)
    : Component(prescaler, clock_gen),
      main_mode_(main_mode),
      sub_mode_(sub_mode),
      target_sat_id_(target_sat_id),
      my_sat_id_(reference_sat_id),
      main_target_direction_b_(main_target_direction_b),
      sub_target_direction_b_(sub_target_direction_b),
      rel_info_(rel_info),
      local_celes_info_(local_celes_info),
      dynamics_(dynamics) {}

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
  main_target_direction_b_ = main_target_direction_b_.CalcNormalizedVector();
  sub_target_direction_b_ = sub_target_direction_b_.CalcNormalizedVector();
  double tmp = libra::InnerProduct(main_target_direction_b_, sub_target_direction_b_);
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
    case RelativeAttitudeControlMode::SUN_POINTING:
      direction_i = local_celes_info_.GetPositionFromSpacecraft_i_m("SUN");
      break;
    case RelativeAttitudeControlMode::EARTH_CENTER_POINTING:
      direction_i = local_celes_info_.GetPositionFromSpacecraft_i_m("EARTH");
      break;
    case RelativeAttitudeControlMode::VELOCITY_DIRECTION_POINTING:
      direction_i = dynamics_.GetOrbit().GetVelocity_i_m_s();
      break;
    case RelativeAttitudeControlMode::ORBIT_NORMAL_POINTING:
      direction_i = libra::OuterProduct(dynamics_.GetOrbit().GetPosition_i_m(), dynamics_.GetOrbit().GetVelocity_i_m_s());
      break;
    default:
      // Not Reached
      break;
  }

  return direction_i.CalcNormalizedVector();
}

libra::Quaternion RelativeAttitudeController::CalcTargetQuaternion(const libra::Vector<3> main_direction_i, const libra::Vector<3> sub_direction_i) {
  // Calc DCM ECI->Target
  libra::Matrix<3, 3> DCM_t2i = CalcDcmFromVectors(main_direction_i, sub_direction_i);
  // Calc DCM Target->body
  libra::Matrix<3, 3> DCM_t2b = CalcDcmFromVectors(main_target_direction_b_, sub_target_direction_b_);
  // Calc DCM ECI->body
  libra::Matrix<3, 3> DCM_i2b = DCM_t2b * DCM_t2i.Transpose();
  // Convert to Quaternion
  return libra::Quaternion::ConvertFromDcm(DCM_i2b);
}

libra::Matrix<3, 3> RelativeAttitudeController::CalcDcmFromVectors(const libra::Vector<3> main_direction, const libra::Vector<3> sub_direction) {
  libra::Matrix<3, 3> dcm = libra::MakeIdentityMatrix<3>();

  // Check vector
  double tmp = libra::InnerProduct(main_direction, sub_direction);
  tmp = std::abs(tmp);
  if (tmp > 0.99999) {
    return dcm;
  }

  // Calc basis vectors
  libra::Vector<3> ex, ey, ez;
  ex = main_direction;
  libra::Vector<3> tmp1 = libra::OuterProduct(ex, sub_direction);
  libra::Vector<3> tmp2 = libra::OuterProduct(tmp1, ex);
  ey = tmp2.CalcNormalizedVector();
  libra::Vector<3> tmp3 = libra::OuterProduct(ex, ey);
  ez = tmp3.CalcNormalizedVector();

  // Generate DCM
  for (int i = 0; i < 3; i++) {
    dcm[i][0] = ex[i];
    dcm[i][1] = ey[i];
    dcm[i][2] = ez[i];
  }
  return dcm;
}

RelativeAttitudeControlMode ConvertStringToRelativeAttitudeControlMode(const std::string mode_name) {
  if (mode_name == "TARGET_SATELLITE_POINTING") {
    return RelativeAttitudeControlMode::TARGET_SATELLITE_POINTING;
  } else if (mode_name == "SUN_POINTING") {
    return RelativeAttitudeControlMode::SUN_POINTING;
  } else if (mode_name == "EARTH_CENTER_POINTING") {
    return RelativeAttitudeControlMode::EARTH_CENTER_POINTING;
  } else if (mode_name == "VELOCITY_DIRECTION_POINTING") {
    return RelativeAttitudeControlMode::VELOCITY_DIRECTION_POINTING;
  } else if (mode_name == "ORBIT_NORMAL_POINTING") {
    return RelativeAttitudeControlMode::ORBIT_NORMAL_POINTING;
  } else {
    // Error
    std::cerr << "RelativeAttitudeControlMode error!" << std::endl;
    return RelativeAttitudeControlMode::TARGET_SATELLITE_POINTING;
  }
}
