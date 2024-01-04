/**
 * @file relative_attitude_sensor.hpp
 * @brief Relative attitude sensor
 */

#ifndef S2E_COMPONENTS_AOCS_RELATIVE_ATTITUDE_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_RELATIVE_ATTITUDE_SENSOR_HPP_

#include <components/base/component.hpp>
#include <library/logger/logger.hpp>
#include <library/randomization/normal_randomization.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

/**
 * @class RelativeAttitudeSensor
 * @brief Relative attitude sensor
 */
class RelativeAttitudeSensor : public Component, public ILoggable {
 public:
  /**
   * @fn RelativeAttitudeSensor
   * @brief Constructor
   */
  RelativeAttitudeSensor(const int prescaler, ClockGenerator* clock_gen, const int target_sat_id, const int reference_sat_id,
                         const RelativeInformation& rel_info, const double standard_deviation_rad);
  /**
   * @fn ~RelativeAttitudeSensor
   * @brief Destructor
   */
  ~RelativeAttitudeSensor();

  // ComponentBase override function
  /**
   * @fn MainRoutine
   * @brief Main routine
   */
  void MainRoutine(int count);

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
  inline libra::Quaternion GetMeasuredQuaternion_rb2tb() const { return measured_quaternion_rb2tb_; }
  inline libra::Vector<3> GetMeasuredEulerAngle_rb2tb_rad() const { return measured_euler_angle_rb2tb_rad_; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;           //!< Target satellite ID
  const int reference_sat_id_;  //!< Reference satellite ID

  // Measured value
  libra::Quaternion measured_quaternion_rb2tb_ = {0.0, 0.0, 0.0, 1.0};  //!< Measured quaternion of target body from reference body
  libra::Vector<3> measured_euler_angle_rb2tb_rad_{
      0.0};  //!< Measured Euler angle of target body in BODY frame [rad], 3-2-1 Euler angle (1: roll, 2: pitch, 3: yaw order)

  // Noises
  libra::NormalRand angle_noise_;      //!< Normal random for magnitude noise
  libra::NormalRand direction_noise_;  //!< Normal random for direction noise

  // References
  const RelativeInformation& rel_info_;  //!< Relative information
};

RelativeAttitudeSensor InitializeRelativeAttitudeSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const int reference_sat_id_input = -1);

#endif  // S2E_COMPONENTS_AOCS_RELATIVE_ATTITUDE_SENSOR_HPP_
