/**
 * @file laser_emitter.hpp
 * @brief Laser Emitter
 */

#ifndef S2E_COMPONENTS_LASER_EMITTER_HPP_
#define S2E_COMPONENTS_LASER_EMITTER_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/vector.hpp>
#include <library/optics/gaussian_beam_base.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"

/**
 * @class LaserEmitter
 * @brief Laser Emitter
 */
class LaserEmitter : public Component, public GaussianBeamBase {
 public:
  /**
   * @fn LaserEmitter
   * @brief Constructor
   */
  LaserEmitter(const int prescaler, ClockGenerator* clock_gen, const Dynamics& dynamics, libra::Vector<3> emitting_direction_c,
               double emission_angle_rad, libra::TranslationFirstDualQuaternion dual_quaternion_c2b, double emission_power_W,
               double radius_beam_waist_m, double rayleigh_length_m, double rayleigh_length_offset_m, double wavelength_m);

  /**
   * @fn ~LaserEmitter
   * @brief Destructor
   */
  ~LaserEmitter() {}

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

  void Update(int count);

  inline libra::Vector<3> GetLaserPosition_i_m() const { return laser_position_i_m_; }
  inline libra::Vector<3> GetEmittingDirection_i() const { return laser_emitting_direction_i_; }

  inline double GetEmissionAngle_rad() const { return emission_angle_rad_; }
  inline double GetRayleighLength_m() const { return rayleigh_length_m_; }
  inline double GetRayleighLengthOffset_m() const { return rayleigh_length_offset_m_; }

 protected:
  libra::Vector<3> emitting_direction_c_{0.0};                 //!< Laser emitting direction vector @ component frame
  double emission_angle_rad_ = 0.0;                            //!< Laser emitting angle from the emitting direction [rad]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from body to component frame

  double rayleigh_length_m_ = 0.0;         //!< Rayleigh length (range) of the laser [m]
  double rayleigh_length_offset_m_ = 0.0;  //!< Rayleigh length (range) position offset of the laser [m]

  libra::Vector<3> laser_position_i_m_{0.0};
  libra::Vector<3> laser_emitting_direction_i_{0.0};
  libra::Vector<3> previous_laser_position_i_m_{0.0};
  libra::Vector<3> previous_laser_emitting_direction_i_{0.0};

  int count_ = 0;

  // Reference
  const Dynamics& dynamics_;
};

LaserEmitter InitializeLaserEmitter(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics& dynamics,
                                    const size_t id = 0);

#endif  // S2E_COMPONENTS_LASER_EMITTER_HPP_
