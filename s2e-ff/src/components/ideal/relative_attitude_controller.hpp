/**
 * @file relative_attitude_controller.hpp
 * @brief Attitude controller for FF satellites to point other satellite.
 */

#ifndef S2E_COMPONENTS_IDEAL_RELATIVE_ATTITUDE_CONTROLLER_HPP_
#define S2E_COMPONENTS_IDEAL_RELATIVE_ATTITUDE_CONTROLLER_HPP_

#include <components/base/component.hpp>
#include <library/logger/logger.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

/**
 * @enum RelativeAttitudeControlMode
 * @brief Relative attitude control mode
 */
enum class RelativeAttitudeControlMode {
  TARGET_SATELLITE_POINTING,    //!< Satellite pointing
  SUN_POINTING,                 //!< Sun pointing
  EARTH_CENTER_POINTING,        //!< Earth center pointing
  VELOCITY_DIRECTION_POINTING,  //!< Velocity direction pointing
  ORBIT_NORMAL_POINTING,        //!< Orbital normal pointing
  NO_CONTROL,                   //!< No control
};

/**
 * @fn ConvertStringToRelativeAttitudeControlMode
 * @brief This function converts mode name written in string to enum RelativeAttitudeControlMode
 * @param [in] mode_name: mode name in string type
 */
RelativeAttitudeControlMode ConvertStringToRelativeAttitudeControlMode(const std::string mode_name);

/**
 * @class RelativeAttitudeController
 * @brief Attitude controller for FF satellites to point other satellite.
 */
class RelativeAttitudeController : public Component, public ILoggable {
 public:
  /**
   * @fn RelativeAttitudeController
   * @brief Constructor
   */
  RelativeAttitudeController(const int prescaler, ClockGenerator* clock_gen, const RelativeAttitudeControlMode main_mode,
                             const RelativeAttitudeControlMode sub_mode, const libra::Vector<3> main_target_direction_b,
                             const libra::Vector<3> sub_target_direction_b, const int target_sat_id, const int reference_sat_id,
                             const RelativeInformation& rel_info, const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics);
  /**
   * @fn ~RelativeAttitudeController
   * @brief Destructor
   */
  ~RelativeAttitudeController();

  // ComponentBase override function
  /**
   * @fn MainRoutine
   * @brief Main routine
   */
  void MainRoutine(int count);
  /**
   * @fn PowerOffRoutine
   * @brief Power off routine
   */
  void PowerOffRoutine();

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

  // Getter

  // Setter
  inline void SetIsCalcEnabled(const bool is_enabled) { is_calc_enabled_ = is_enabled; }
  inline void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  bool is_calc_enabled_ = true;               //!< Calculation enabled
  RelativeAttitudeControlMode main_mode_;     //!< Control mode for main axis
  RelativeAttitudeControlMode sub_mode_;      //!< Control mode for sub axis
  int target_sat_id_;                         //!< Target satellite ID
  int my_sat_id_;                             //!< Satellite ID of the satellite which mounts this components

  libra::Vector<3> main_target_direction_b_;  //!< Pointing main target direction on body frame
  libra::Vector<3> sub_target_direction_b_;   //!< Pointing sub target direction on body frame

  // References
  const RelativeInformation& rel_info_;                //!< Relative information
  const LocalCelestialInformation& local_celes_info_;  //!< Local celestial information
  const Dynamics& dynamics_;                           //!< Dynamics

  // Internal functions
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize(void);
  /**
   * @fn CalcTargetDirection_i
   * @brief Calculate target direction in ECI frame from attitude control mode
   */
  libra::Vector<3> CalcTargetDirection_i(RelativeAttitudeControlMode mode);
  /**
   * @fn CalcTargetQuaternion
   * @brief Calculate target quaternion from ECI frame information
   */
  libra::Quaternion CalcTargetQuaternion(const libra::Vector<3> main_direction_i, const libra::Vector<3> sub_direction_i);
  /**
   * @fn CalcDcmFromVectors
   * @brief Calculate Direction Cosine Matrix from two vectors
   */
  libra::Matrix<3, 3> CalcDcmFromVectors(const libra::Vector<3> main_direction,
                                         const libra::Vector<3> sub_direction);  // TODO: move to core's library
};

#endif  // S2E_COMPONENTS_IDEAL_RELATIVE_ATTITUDE_CONTROLLER_HPP_
