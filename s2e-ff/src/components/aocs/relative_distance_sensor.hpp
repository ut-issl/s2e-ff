/**
 * @file relative_distance_sensor.hpp
 * @brief Relative distance sensor
 */

#ifndef S2E_COMPONENTS_AOCS_RELATIVE_DISTANCE_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_RELATIVE_DISTANCE_SENSOR_HPP_

#include <components/base/component.hpp>
#include <components/base/sensor.hpp>
#include <library/logger/logger.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

/**
 * @class RelativeDistanceSensor
 * @brief Relative distance sensor
 */
class RelativeDistanceSensor : public Component, public Sensor<1>, public ILoggable {
 public:
  /**
   * @fn RelativeDistanceSensor
   * @brief Constructor
   */
  RelativeDistanceSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id, const int reference_sat_id,
                         const RelativeInformation& rel_info);
  /**
   * @fn ~RelativeDistanceSensor
   * @brief Destructor
   */
  ~RelativeDistanceSensor();

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
  inline double GetMeasuredDistance_m() const { return measured_distance_bw_ref_target_m_[0]; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;           //!< Target satellite ID
  const int reference_sat_id_;  //!< Reference satellite ID

  // Measured value
  libra::Vector<1> measured_distance_bw_ref_target_m_{0.0};  //!< Measured distance [m]

  // References
  const RelativeInformation& rel_info_;  //!< Relative information
};

#endif  // S2E_COMPONENTS_AOCS_RELATIVE_DISTANCE_SENSOR_HPP_
