/**
 * @file laser_distance_meter.hpp
 * @brief Laser distance meter
 */

#ifndef S2E_COMPONENTS_LASER_DISTANCE_METER_HPP_
#define S2E_COMPONENTS_LASER_DISTANCE_METER_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"
#include "../../simulation/case/ff_inter_spacecraft_communication.hpp"

/**
 * @class LaserDistanceMeter
 * @brief Relative distance sensor
 */
class LaserDistanceMeter : public Component, public ILoggable {
 public:
  /**
   * @fn LaserDistanceMeter
   * @brief Constructor
   */
  LaserDistanceMeter(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics& dynamics,
                     const FfInterSpacecraftCommunication& inter_spacecraft_communication, const size_t id = 0);
  /**
   * @fn ~LaserDistanceMeter
   * @brief Destructor
   */
  ~LaserDistanceMeter() {}

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

  inline double GetObservedDistance_m() const { return observed_distance_m_; }
  inline bool GetIsReflected() const { return is_reflected_; }

 protected:
  libra::Vector<3> laser_emitting_direction_c_;                //!< Laser emitting direction @ component frame
  double emission_angle_rad_;                                  //!< Emission half angle from the normal direction [rad]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from component to body frame

  bool is_reflected_ = false;         //!< Flag to detect reflected light
  double observed_distance_m_ = 0.0;  //!< Observed direction
  size_t laser_id_ = 0;

  // Reference
  const Dynamics& dynamics_;
  const FfInterSpacecraftCommunication& inter_spacecraft_communication_;

  double CalcDistanceBwPointAndLine(libra::Vector<3> point_position, libra::Vector<3> line_start_position, libra::Vector<3> line_direction);

  void Initialize(const std::string file_name, const size_t id = 0);
};

#endif  // S2E_COMPONENTS_LASER_DISTANCE_METER_HPP_
