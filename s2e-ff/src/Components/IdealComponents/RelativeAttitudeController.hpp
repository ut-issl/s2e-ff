#pragma once

#include <Component/Abstract/ComponentBase.h>
#include <Interface/LogOutput/Logger.h>
#include <RelativeInformation/RelativeInformation.h>

enum class RelativeAttitudeControlMode {
  TARGET_SATELLITE_POINTING,
  SUN_POINTING,
  BODY_CENTER_POINTING,
  VELOCITY_DIRECTION_POINTING,
  ORBIT_NORMAL_POINTING,
  NO_CONTROL,
};

RelativeAttitudeControlMode ConvertStringToRelativeAttitudeControlMode(const std::string mode_name);

class RelativeAttitudeController : public ComponentBase, public ILoggable {
 public:
  RelativeAttitudeController(const int prescaler, ClockGenerator* clock_gen, const RelativeAttitudeControlMode main_mode,
                             const RelativeAttitudeControlMode sub_mode, const libra::Vector<3> main_target_direction_b,
                             const libra::Vector<3> sub_target_direction_b, const int target_sat_id, const int reference_sat_id,
                             const RelativeInformation& rel_info, const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics);
  ~RelativeAttitudeController();

  // ComponentBase override function
  void MainRoutine(int count);
  void PowerOffRoutine();

  // ILogabble override function
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter

  // Setter
  inline void SetIsCalcEnabled(const bool is_enabled) { is_calc_enabled_ = is_enabled; }
  inline void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  bool is_calc_enabled_ = true;
  RelativeAttitudeControlMode main_mode_;
  RelativeAttitudeControlMode sub_mode_;
  int target_sat_id_;
  int my_sat_id_;

  libra::Vector<3> main_target_direction_b_;  //!< Pointing main target on body frame
  libra::Vector<3> sub_target_direction_b_;   //!< Pointing main target on body frame

  const RelativeInformation& rel_info_;
  const LocalCelestialInformation& local_celes_info_;
  const Dynamics& dynamics_;

  // Internal functions
  void Initialize(void);
  libra::Vector<3> CalcTargetDirection_i(RelativeAttitudeControlMode mode);
  libra::Quaternion CalcTargetQuaternion(const libra::Vector<3> main_direction_i, const libra::Vector<3> sub_direction_i);
  libra::Matrix<3, 3> CalcDcmFromVectors(const libra::Vector<3> main_direction,
                                         const libra::Vector<3> sub_direction);  // TODO: move to core's library
};
