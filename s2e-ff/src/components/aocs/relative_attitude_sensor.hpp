/**
 * @file relative_attitude_sensor.hpp
 * @brief Relative attitude sensor
 */

#ifndef S2E_COMPONENTS_AOCS_RELATIVE_ATTITUDE_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_RELATIVE_ATTITUDE_SENSOR_HPP_

#include <components/base/component.hpp>
#include <components/base/sensor.hpp>
#include <library/logger/logger.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

/**
 * @class RelativeAttitudeSensor
 * @brief Relative position sensor
 */
class RelativeAttitudeSensor : public Component, public Sensor<3>, public ILoggable {
 public:
  /**
   * @fn RelativeAttitudeSensor
   * @brief Constructor
   */
  RelativeAttitudeSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id, const int reference_sat_id,
                         const RelativeInformation& rel_info, const Dynamics& dynamics);
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
  inline libra::Quaternion GetMeasuredTargetQuaternion_b_m() const { return measured_target_attitude_b_quaternion_; }
  inline libra::Vector<3> GetMeasuredTargetAttitude_b_m() const { return measured_target_attitude_b_rad_; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;           //!< Target satellite ID
  const int reference_sat_id_;  //!< Reference satellite ID

  // Measured value
  libra::Quaternion measured_target_attitude_b_quaternion_;
  libra::Vector<3> measured_target_attitude_b_rad_{0.0};  //!< Measured attitude in BODY frame [m]

  // References
  const RelativeInformation& rel_info_;  //!< Relative information
  const Dynamics& dynamics_;             //!< Dynamics
};

#endif  // S2E_COMPONENTS_AOCS_RELATIVE_ATTITUDE_SENSOR_HPP_
