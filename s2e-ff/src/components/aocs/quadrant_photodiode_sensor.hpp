/**
 * @file quadrant_photodiode_sensor.hpp
 * @brief Quadrant photodiode (QPD) sensor
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
 * @brief Quadrant photodiode sensor
 * @note  This component not only calculate the QPD sensor output values, but also calculate position displacements.
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

  inline double GetYAxisDisplacementTrue_m() const { return y_axis_displacement_true_m_; }
  inline double GetZAxisDisplacementTrue_m() const { return z_axis_displacement_true_m_; }
  inline double GetObservedYAxisDisplacement_m() const { return observed_y_axis_displacement_m_; }
  inline double GetObservedZAxisDisplacement_m() const { return observed_z_axis_displacement_m_; }
  inline bool GetIsReceivedLaser() const { return is_received_laser_; }

  /**
   * @enum QpdPositionDeterminationDirection
   * @brief Type of the quadrant photodiode sensor output value
   */
  typedef enum {
    yAxisDirection = 0,  //!< y-axis direction
    zAxisDirection,      //!< z-axis direction
  } QpdPositionDeterminationDirection;

 protected:
  double qpd_laser_receivable_angle_rad_;                      //!< laser receivable half angle from the normal direction [rad]
  double qpd_sensor_output_voltage_threshold_V_;               //!< Quadrant photodiode sensor output voltage threshold [V]
  double qpd_sensor_radius_m_;                                 //!< Quadrant photodiode sensor radius [m]
  double qpd_sensor_integral_step_m_;                          //!< Integral step to calculate the output value of the quadrant photodiode sensor [m]
  double qpd_sensor_position_determination_threshold_m_;       //!< Position determination threshold using the quadrant photodiode sensor [m]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from component to body frame

  bool is_received_laser_ = false;  //!< Flag to detect laser received
  double distance_true_m_ = 0.0;
  double y_axis_displacement_true_m_ = 0.0;
  double z_axis_displacement_true_m_ = 0.0;

  libra::Vector<3> x_axis_direction_c_{0.0};  //!< x-axis direction in the component coordinate system
  libra::Vector<3> y_axis_direction_c_{0.0};  //!< x-axis direction in the component coordinate system
  libra::Vector<3> z_axis_direction_c_{0.0};  //!< x-axis direction in the component coordinate system

  double qpd_sensor_output_y_axis_V_;            //!< Quadrant photodiode sensor output value corresponding to the y-axis direction [V]
  double qpd_sensor_output_z_axis_V_;            //!< Quadrant photodiode sensor output value corresponding to the y-axis direction [V]
  double qpd_sensor_output_sum_V_;               //!< Quadrant photodiode sensor output value corresponding to the sum of the light intensity [V]
  double observed_y_axis_displacement_m_ = 0.0;  //!< Observed displacement in the y-axis direction [m]
  double observed_z_axis_displacement_m_ = 0.0;  //!< Observed displacement in the z-axis direction [m]

  std::vector<double> qpd_displacement_reference_list_m_;
  std::vector<double> qpd_ratio_y_reference_list_;
  std::vector<double> qpd_ratio_z_reference_list_;

  // Reference
  const Dynamics& dynamics_;
  const FfInterSpacecraftCommunication& inter_spacecraft_communication_;

  libra::Vector<3> CalcLaserReceivedPosition(const libra::Vector<3> point_position, const libra::Vector<3> origin_position,
                                             const libra::Vector<3> plane_normal_direction, const libra::Vector<3> point_line_direction);
  double CalcDisplacement(const libra::Vector<3> point_position, const libra::Vector<3> origin_position,
                          const libra::Vector<3> displacement_direction);

  void CalcSensorOutput(LaserEmitter* laser_emitter, const double distance_from_beam_waist_m, const double qpd_horizontal_displacement_m,
                        const double qpd_vertical_displacement_m);
  double DeterminePositionDisplacement(const QpdPositionDeterminationDirection determination_direction, const double qpd_sensor_output_V,
                                       const double qpd_sensor_output_sum_V, const std::vector<double>& qpd_ratio_reference_list);
  double CalcSign(const double input_value, const double threshold);

  void Initialize(const std::string file_name, const size_t id = 0);
};

#endif  // S2E_COMPONENTS_QUADRANT_PHOTODIODE_SENSOR_HPP_
