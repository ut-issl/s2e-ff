/**
 * @file laser_distance_meter.hpp
 * @brief Laser distance meter
 */

#ifndef S2E_COMPONENTS_QUADRANT_PHOTODIODE_SENSOR_HPP_
#define S2E_COMPONENTS_QUADRANT_PHOTODIODE_SENSOR_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"
#include "../../simulation/case/ff_inter_spacecraft_communication.hpp"

/**
 * @class QuadrantPhotodiodeSensor
 * @brief Relative distance sensor
 */
class QuadrantPhotodiodeSensor : public Component, public ILoggable {
 public:
  /**
   * @fn QuadrantPhotodiodeSensor
   * @brief Constructor
   */
  QuadrantPhotodiodeSensor(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics& dynamics,
                           const FfInterSpacecraftCommunication& inter_spacecraft_communication, const size_t id = 0);
  /**
   * @fn ~QuadrantPhotodiodeSensor
   * @brief Destructor
   */
  ~QuadrantPhotodiodeSensor() {}

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

  inline double GetActualHorizontalDisplacement_m() const { return actual_horizontal_displacement_m_; }
  inline double GetActualVerticalDisplacement_m() const { return actual_vertical_displacement_m_; }
  inline double GetObservedHorizontalDisplacement_m() const { return observed_horizontal_displacement_m_; }
  inline double GetObservedVerticalDisplacement_m() const { return observed_vertical_displacement_m_; }
  inline bool GetIsReflected() const { return is_received_laser_; }

 protected:
  libra::Vector<3> qpd_horizontal_direction_c_;                //!< Quadrant photodiode horizontal direction @ component frame
  libra::Vector<3> qpd_vertical_direction_c_;                  //!< Quadrant photodiode vertical direction @ component frame
  libra::Vector<3> qpd_normal_direction_c_;                    //!< Quadrant photodiode normal direction @ component frame
  double qpd_laser_received_angle_rad_;                        //!< laser received half angle from the normal direction [rad]
  double qpd_sensor_radius_m_;                                 //!< Quadrant photodiode sensor radius [m]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from component to body frame

  bool is_received_laser_ = false;  //!< Flag to detect reflected light
  double actual_distance_m_ = 0.0;
  double actual_horizontal_displacement_m_ = 0.0;
  double actual_vertical_displacement_m_ = 0.0;
  double observed_horizontal_displacement_m_ = 0.0;  //!< Observed horizontal displacement
  double observed_vertical_displacement_m_ = 0.0;    //!< Observed vertical displacement

  libra::Vector<3> qpd_sensor_output_V_;

  std::vector<double> qpd_displacement_reference_list_mm_;
  std::vector<double> qpd_ratio_y_ref_reference_list_;
  std::vector<double> qpd_ratio_z_ref_reference_list_;

  // Reference
  const Dynamics& dynamics_;
  const FfInterSpacecraftCommunication& inter_spacecraft_communication_;

  double CalcDistanceBwPointAndLine(libra::Vector<3> point_position, libra::Vector<3> line_start_position, libra::Vector<3> line_direction);
  libra::Vector<3> CalcLaserReceivedPosition(libra::Vector<3> point_position, libra::Vector<3> origin_position,
                                             libra::Vector<3> plane_normal_direction, libra::Vector<3> point_line_direction);
  double CalcDisplacement(libra::Vector<3> point_position, libra::Vector<3> origin_position, libra::Vector<3> displacement_direction);

  void CalcSensorOutput(double laser_power_W, double laser_beam_radius, double qpd_horizontal_displacement_m, double qpd_vertical_displacement_m);

  void Initialize(const std::string file_name, const size_t id = 0);
};

#endif  // S2E_COMPONENTS_LASER_DISTANCE_METER_HPP_
