/**
 * @file qpd_positioning_sensor.hpp
 * @brief Quadrant photodiode (QPD) positioning sensor
 */

#ifndef S2E_COMPONENTS_QPD_POSITIONING_SENSOR_HPP_
#define S2E_COMPONENTS_QPD_POSITIONING_SENSOR_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/normal_randomization.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"
#include "../../simulation/case/ff_inter_spacecraft_communication.hpp"

/**
 * @class QpdPositioningSensor
 * @brief Quadrant photodiode sensor
 * @note  This component not only calculate the QPD sensor output values, but also calculate position displacements.
 */
class QpdPositioningSensor : public Component, public ILoggable {
 public:
  /**
   * @fn QpdPositioningSensor
   * @brief Constructor
   */
  QpdPositioningSensor(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics& dynamics,
                       const FfInterSpacecraftCommunication& inter_spacecraft_communication, const size_t id = 0);
  /**
   * @fn ~QpdPositioningSensor
   * @brief Destructor
   */
  ~QpdPositioningSensor() {}

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

  void SetErrorCompensatedCoefficient(double line_of_sight_distance_m);

  double GetObservedYAxisDisplacementAfterCompensation_m();
  double GetObservedZAxisDisplacementAfterCompensation_m();
  double GetObservedYAxisDisplacementAfterCompensation_m(double line_of_sight_distance_m);
  double GetObservedZAxisDisplacementAfterCompensation_m(double line_of_sight_distance_m);

  inline bool GetIsReceivedLaser() const { return is_received_laser_; }

 protected:
  double qpd_sensor_sensitivity_coefficient_V_W_;              //!< Sensitivity coefficient of the quadrant photodiode sensor [V/W]
  double qpd_laser_receivable_angle_rad_;                      //!< laser receivable half angle from the normal direction [rad]
  double qpd_sensor_output_voltage_threshold_V_;               //!< Quadrant photodiode sensor output voltage threshold [V]
  double qpd_sensor_radius_m_;                                 //!< Quadrant photodiode sensor radius [m]
  double qpd_sensor_integral_step_m_;                          //!< Integral step to calculate the output value of the quadrant photodiode sensor [m]
  double qpd_positioning_threshold_m_;                         //!< Position determination threshold using the quadrant photodiode sensor [m]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from component to body frame

  bool is_received_laser_ = false;  //!< Flag to detect laser received
  double distance_true_m_ = 0.0;
  double y_axis_displacement_true_m_ = 0.0;
  double z_axis_displacement_true_m_ = 0.0;

  libra::Vector<3> x_axis_direction_c_{0.0};  //!< x-axis direction in the component coordinate system
  libra::Vector<3> y_axis_direction_c_{0.0};  //!< y-axis direction in the component coordinate system
  libra::Vector<3> z_axis_direction_c_{0.0};  //!< z-axis direction in the component coordinate system

  // This quadrant photodiode sensor is modeled after Thorlabs' PDQ80A product.
  // Therefore, the acquired values are not the raw values of the four photodiodes but the following three values:
  double qpd_sensor_output_y_axis_V_ = 0.0;  //!< Quadrant photodiode sensor output value corresponding to the y-axis direction: E_y [V]
  double qpd_sensor_output_z_axis_V_ = 0.0;  //!< Quadrant photodiode sensor output value corresponding to the y-axis direction: E_z [V]
  double qpd_sensor_output_sum_V_ = 0.0;     //!< Quadrant photodiode sensor output value corresponding to the sum of the light intensity: E_sum [V]

  // Noise parameters
  libra::NormalRand qpd_sensor_output_random_noise_;  //!< Normal random noise for QPD sensor output value
  double qpd_standard_deviation_scale_factor_;        //!< Scale factor of the standard deviation: Coefficient to express position dependency
  double qpd_standard_deviation_constant_V_;          //!< Constant value of the standard deviation, which is constant regardless of its position

  double observed_y_axis_displacement_m_ = 0.0;  //!< Observed displacement in the y-axis direction [m]
  double observed_z_axis_displacement_m_ = 0.0;  //!< Observed displacement in the z-axis direction [m]

  // The following arrays are required to observe the position displacements.
  std::vector<double> qpd_sensor_displacement_list_m_;
  std::vector<double>
      qpd_sensor_voltage_ratio_y_list_;  //!< List of `qpd_sensor_output_y_axis_V / qpd_sensor_output_sum_V` at each point on the y-axis.
  std::vector<double>
      qpd_sensor_voltage_ratio_z_list_;  //!< List of `qpd_sensor_output_z_axis_V / qpd_sensor_output_sum_V` at each point on the z-axis.

  // The following arrays are required to compensate observed position displacement errors.
  std::vector<double> line_of_sight_distance_list_m_;
  std::vector<double> error_compensated_coefficient_list_;  //!< This coefficients compensate the errors of the observed position displacement.
  double error_compensated_coefficient_ = 1.0;              //!< Coefficient to compensate the errors of the observed position displacement.

  size_t qpd_positioning_sensor_id_ = 0;

  // Reference
  const Dynamics& dynamics_;
  const FfInterSpacecraftCommunication& inter_spacecraft_communication_;

  libra::Vector<3> CalcLaserReceivedPosition(const libra::Vector<3> point_position, const libra::Vector<3> origin_position,
                                             const libra::Vector<3> plane_normal_direction, const libra::Vector<3> point_line_direction);
  double CalcDisplacement(const libra::Vector<3> point_position, const libra::Vector<3> origin_position,
                          const libra::Vector<3> displacement_direction);

  void CalcSensorOutput(LaserEmitter* laser_emitter, const double qpd_laser_distance_m, const double laser_rayleigh_length_offset_m,
                        const double qpd_y_axis_displacement_m, const double qpd_z_axis_displacement_m);
  double ObservePositionDisplacement(const double qpd_sensor_output_polarization, const double qpd_sensor_output_V,
                                     const double qpd_sensor_output_sum_V, const std::vector<double>& qpd_ratio_reference_list);
  double CalcSign(const double input_value, const double threshold);
  double CalcStandardDeviation(const double sensor_output_derivative, const double qpd_laser_distance_m);

  void Initialize(const std::string file_name, const size_t id = 0);
};

QpdPositioningSensor InitializeQpdPositioningSensor(ClockGenerator* clock_gen, const std::string file_name, double compo_step_time_s,
                                                    const Dynamics& dynamics, const FfInterSpacecraftCommunication& inter_spacecraft_communication,
                                                    const size_t id = 0);

#endif  // S2E_COMPONENTS_QPD_POSITIONING_SENSOR_HPP_
