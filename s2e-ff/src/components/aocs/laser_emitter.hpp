/**
 * @file laser_emitter.hpp
 * @brief Laser Emitter
 */

#ifndef S2E_COMPONENTS_LASER_EMITTER_HPP_
#define S2E_COMPONENTS_LASER_EMITTER_HPP_

#include <dynamics/dynamics.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/vector.hpp>
#include <library/optics/gaussian_beam_base.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"

/**
 * @class LaserEmitter
 * @brief Laser Emitter
 */
class LaserEmitter : public GaussianBeamBase {
 public:
  /**
   * @fn LaserEmitter
   * @brief Constructor
   */
  LaserEmitter(const Dynamics& dynamics, libra::Vector<3> emitting_direction_c, double emission_angle_rad,
               libra::TranslationFirstDualQuaternion dual_quaternion_c2b, double emission_power_W, double radius_beam_waist_m,
               double rayleigh_length_m, double rayleigh_length_offset_m, double wavelength_m);

  /**
   * @fn ~LaserEmitter
   * @brief Destructor
   */
  ~LaserEmitter() {}

  inline libra::Vector<3> GetLaserPosition_i_m() const {
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_.GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    return dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
  }

  inline libra::Vector<3> GetEmittingDirection_i() const {
    // Body -> Inertial frame
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_.GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_.GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    libra::Vector<3> laser_position_i_m = dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
    libra::Vector<3> emitting_direction_i = dual_quaternion_c2i.TransformVector(emitting_direction_c_);
    emitting_direction_i -= laser_position_i_m;

    return emitting_direction_i;
  }

  inline double GetEmissionAngle_rad() const { return emission_angle_rad_; }
  inline double GetRayleighLength_m() const { return rayleigh_length_m_; }
  inline double GetRayleighLengthOffset_m() const { return rayleigh_length_offset_m_; }

 protected:
  libra::Vector<3> emitting_direction_c_{0.0};                 //!< Laser emitting direction vector @ component frame
  double emission_angle_rad_ = 0.0;                            //!< Laser emitting angle from the emitting direction [rad]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from body to component frame

  double rayleigh_length_m_ = 0.0;         //!< Rayleigh length (range) of the laser [m]
  double rayleigh_length_offset_m_ = 0.0;  //!< Rayleigh length (range) position offset of the laser [m]

  // Reference
  const Dynamics& dynamics_;
};

LaserEmitter InitializeLaserEmitter(const std::string file_name, const Dynamics& dynamics, const size_t id = 0);

#endif  // S2E_COMPONENTS_LASER_EMITTER_HPP_
