/**
 * @file relative_velocity_sensor.hpp
 * @brief Relative velocity sensor
 */

#ifndef S2E_COMPONENTS_AOCS_RELATIVE_VELOCITY_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_RELATIVE_VELOCITY_SENSOR_HPP_

#include <components/base/component.hpp>
#include <components/base/sensor.hpp>
#include <library/logger/logger.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

/**
 * @enum RelativeVelocitySensorErrorFrame
 * @brief Error definition frame
 */
enum class RelativeVelocitySensorErrorFrame {
  INERTIAL,  //!< Inertial frame
  RTN        //!< RTN frame
};

/**
 * @class RelativeVelocitySensor
 * @brief Relative velocity sensor
 */
class RelativeVelocitySensor : public Component, public Sensor<3>, public ILoggable {
 public:
  /**
   * @fn RelativeVelocitySensor
   * @brief Constructor
   */
  RelativeVelocitySensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id, const int reference_sat_id,
                         const RelativeVelocitySensorErrorFrame error_frame, const RelativeInformation& rel_info, const Dynamics& dynamics);
  /**
   * @fn ~RelativeVelocitySensor
   * @brief Destructor
   */
  ~RelativeVelocitySensor();

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
  inline libra::Vector<3> GetMeasuredTargetVelocity_i_m_s() const { return measured_target_velocity_i_m_s_; }
  inline libra::Vector<3> GetMeasuredTargetVelocity_rtn_m_s() const { return measured_target_velocity_rtn_m_s_; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;                             //!< Target satellite ID
  const int reference_sat_id_;                    //!< Reference satellite ID
  RelativeVelocitySensorErrorFrame error_frame_;  //!< Error definition frame

  // Measured value
  libra::Vector<3> measured_target_velocity_i_m_s_{0.0};    //!< Measured velocity in ECI frame [m/s]
  libra::Vector<3> measured_target_velocity_rtn_m_s_{0.0};  //!< Measured velocity in RTN frame [m/s]

  // References
  const RelativeInformation& rel_info_;  //!< Relative information
  const Dynamics& dynamics_;             //!< Dynamics
};

/**
 * @fn InitializeRelativeVelocitySensor
 * @brief Initialize function
 */
RelativeVelocitySensor InitializeRelativeVelocitySensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics,
                                                        const int reference_sat_id_input = -1);

#endif  // S2E_COMPONENTS_AOCS_RELATIVE_VELOCITY_SENSOR_HPP_
