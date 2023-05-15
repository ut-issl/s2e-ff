/**
 * @file relative_position_sensor.hpp
 * @brief Relative position sensor
 */

#ifndef S2E_COMPONENTS_AOCS_RELATIVE_POSITION_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_RELATIVE_POSITION_SENSOR_HPP_

#include <components/base/component.hpp>
#include <components/base/sensor.hpp>
#include <library/logger/logger.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

/**
 * @enum RelativePositionSensorErrorFrame
 * @brief Error definition frame
 */
enum class RelativePositionSensorErrorFrame {
  INERTIAL,  //!< Inertial frame
  RTN,       //!< RTN frame
  BODY       //!< BODY fixed frame
};

/**
 * @class RelativePositionSensor
 * @brief Relative position sensor
 */
class RelativePositionSensor : public Component, public Sensor<3>, public ILoggable {
 public:
  /**
   * @fn RelativePositionSensor
   * @brief Constructor
   */
  RelativePositionSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id, const int reference_sat_id,
                         const RelativePositionSensorErrorFrame error_frame, const RelativeInformation& rel_info, const Dynamics& dynamics);
  /**
   * @fn ~RelativePositionSensor
   * @brief Destructor
   */
  ~RelativePositionSensor();

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
  inline libra::Vector<3> GetMeasuredTargetPosition_i_m() const { return measured_target_position_i_m_; }
  inline libra::Vector<3> GetMeasuredTargetPosition_rtn_m() const { return measured_target_position_rtn_m_; }
  inline libra::Vector<3> GetMeasuredTargetPosition_b_m() const { return measured_target_position_body_m_; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;                             //!< Target satellite ID
  const int reference_sat_id_;                    //!< Reference satellite ID
  RelativePositionSensorErrorFrame error_frame_;  //!< Error definition frame

  // Measured value
  libra::Vector<3> measured_target_position_i_m_{0.0};     //!< Measured position in ECI frame [m]
  libra::Vector<3> measured_target_position_rtn_m_{0.0};   //!< Measured position in RTN frame [m]
  libra::Vector<3> measured_target_position_body_m_{0.0};  //!< Measured position in BODY frame [m]

  // References
  const RelativeInformation& rel_info_;  //!< Relative information
  const Dynamics& dynamics_;             //!< Dynamics
};

#endif  // S2E_COMPONENTS_AOCS_RELATIVE_POSITION_SENSOR_HPP_
