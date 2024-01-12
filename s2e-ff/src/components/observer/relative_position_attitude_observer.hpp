/**
 * @file relative_position_attitude_observer.hpp
 * @brief Relative Position and Attitude Observer
 */

#ifndef S2E_COMPONENTS_RELATIVE_POSITION_ATTITUDE_OBSERVER_HPP_
#define S2E_COMPONENTS_RELATIVE_POSITION_ATTITUDE_OBSERVER_HPP_

#include <components/base/component.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>

#include "../aocs/laser_distance_meter.hpp"
#include "../aocs/qpd_positioning_sensor.hpp"

const size_t kLaserDistanceMetersNumber = 3;
const size_t kQpdPositioningSensorsNumber = 2;

/**
 * @class RelativePositionAttitudeObserver
 * @brief Relative Position and Attitude Observer
 */
class RelativePositionAttitudeObserver : public Component, public ILoggable {
 public:
  /**
   * @fn RelativePositionAttitudeObserver
   * @brief Constructor
   */
  RelativePositionAttitudeObserver(const int prescaler, ClockGenerator* clock_gen, std::vector<LaserDistanceMeter*>& laser_distance_meters,
                                   std::vector<QpdPositioningSensor*>& qpd_positioning_sensors);
  /**
   * @fn ~RelativePositionAttitudeObserver
   * @brief Destructor
   */
  ~RelativePositionAttitudeObserver() {}

  // ComponentBase override function
  /**
   * @fn MainRoutine
   * @brief Main routine
   */
  void MainRoutine(int count) override;

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

  inline libra::Vector<3> GetObservedRelativePosition_m() const { return observed_relative_position_m_; }
  inline libra::Vector<3> GetObservedRelativeEulerAngle_rad() const { return observed_relative_euler_angle_rad_; }

 protected:
  std::vector<LaserDistanceMeter*> laser_distance_meters_;
  std::vector<QpdPositioningSensor*> qpd_positioning_sensors_;

  libra::Vector<3> observed_relative_position_m_{0.0};
  libra::Vector<3> observed_relative_euler_angle_rad_{0.0};

  double component_position_y_axis_m_ = 0.5;
  double component_position_z_axis_m_ = 0.5;

  // Funbstions
  void ObserveRelativePositionAttitude();
};

#endif  // S2E_COMPONENTS_RELATIVE_POSITION_ATTITUDE_ANALYZER_HPP_
