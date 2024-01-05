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
  inline double GetObservedHorizontalDisplacement_m() const { return determined_horizontal_displacement_m_; }
  inline double GetObservedVerticalDisplacement_m() const { return determined_vertical_displacement_m_; }
  inline bool GetIsReflected() const { return is_received_laser_; }

  /**
   * @enum QpdSensorOutputValueType
   * @brief Type of the quadrant photodiode sensor output value
   */
  typedef enum {
    VOLTAGE_HORIZONTAL = 0,  //!< Value corresponding to the horizontal displacement
    VOLTAGE_VERTICAL,        //!< Value corresponding to the vertical displacement
    VOLTAGE_SUM              //!< Value corresponding to the sum of the light intensity
  } QpdSensorOutputValueType;

  /**
   * @enum QpdPositionDeterminationDirection
   * @brief Type of the quadrant photodiode sensor output value
   */
  typedef enum {
    DIRECTION_HORIZONTAL = 0,  //!< Horizontal direction
    DIRECTION_VERTICAL,        //!< Vertical direction
  } QpdPositionDeterminationDirection;

 protected:
  libra::Vector<3> qpd_horizontal_direction_c_;                //!< Quadrant photodiode horizontal direction @ component frame
  libra::Vector<3> qpd_vertical_direction_c_;                  //!< Quadrant photodiode vertical direction @ component frame
  libra::Vector<3> qpd_normal_direction_c_;                    //!< Quadrant photodiode normal direction @ component frame
  double qpd_laser_received_angle_rad_;                        //!< laser received half angle from the normal direction [rad]
  double qpd_sensor_output_voltage_threshold_V_;               //!< Quadrant photodiode sensor output voltage threshold [V]
  double qpd_sensor_radius_m_;                                 //!< Quadrant photodiode sensor radius [m]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from component to body frame

  bool is_received_laser_ = false;  //!< Flag to detect laser received
  double actual_distance_m_ = 0.0;
  double actual_horizontal_displacement_m_ = 0.0;
  double actual_vertical_displacement_m_ = 0.0;

  libra::Vector<3> qpd_sensor_output_V_;               //!< Observed quadrant photodiode sensor output
  double determined_horizontal_displacement_m_ = 0.0;  //!< Determined horizontal displacement
  double determined_vertical_displacement_m_ = 0.0;    //!< Determined vertical displacement

  std::vector<double> qpd_displacement_reference_list_m_;
  std::vector<double> qpd_ratio_y_reference_list_;
  std::vector<double> qpd_ratio_z_reference_list_;

  // Reference
  const Dynamics& dynamics_;
  const FfInterSpacecraftCommunication& inter_spacecraft_communication_;

  double CalcDistanceBwPointAndLine(libra::Vector<3> point_position, libra::Vector<3> line_start_position, libra::Vector<3> line_direction);
  libra::Vector<3> CalcLaserReceivedPosition(libra::Vector<3> point_position, libra::Vector<3> origin_position,
                                             libra::Vector<3> plane_normal_direction, libra::Vector<3> point_line_direction);
  double CalcDisplacement(libra::Vector<3> point_position, libra::Vector<3> origin_position, libra::Vector<3> displacement_direction);

  void CalcSensorOutput(double laser_power_W, double laser_beam_radius, double qpd_horizontal_displacement_m, double qpd_vertical_displacement_m);
  double DeterminePositionDisplacement(QpdPositionDeterminationDirection determination_direction, libra::Vector<3> qpd_sensor_output_V);
  double CalcSgn(double input_value, double threshold);

  void Initialize(const std::string file_name, const size_t id = 0);
};

#endif  // S2E_COMPONENTS_LASER_DISTANCE_METER_HPP_
