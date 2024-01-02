/**
 * @file corner_cube_reflector.hpp
 * @brief Corner cube reflector
 */

#ifndef S2E_COMPONENTS_LASER_EMITTER_HPP_
#define S2E_COMPONENTS_LASER_EMITTER_HPP_

#include <dynamics/dynamics.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/vector.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"

/**
 * @class LaserEmitter
 * @brief Corner Cube Reflector
 */
class LaserEmitter {
 public:
  /**
   * @fn LaserEmitter
   * @brief Constructor
   */
  LaserEmitter() : dynamics_(nullptr) {}
  /**
   * @fn LaserEmitter
   * @brief Constructor
   */
  LaserEmitter(const std::string file_name, const Dynamics* dynamics, const size_t id = 0) : dynamics_(dynamics) { Initialize(file_name, id); }
  /**
   * @fn ~LaserEmitter
   * @brief Destructor
   */
  ~LaserEmitter() {}

  inline libra::Vector<3> GetLaserPosition_i_m() const {
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    return dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
  }

  inline libra::Vector<3> GetEmittingDirection_i() const {
    // Body -> Inertial frame
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    libra::Vector<3> laser_position_i_m = dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
    libra::Vector<3> emitting_direction_i = dual_quaternion_c2i.TransformVector(emitting_direction_c_);
    emitting_direction_i -= laser_position_i_m;

    return emitting_direction_i;
  }

  inline double GetEmissionAngle_rad() const { return emission_angle_rad_; }

  inline double GetBeamWaist_mm() const { return laser_beam_waist_mm_; }
  inline double GetRayleighRange_m() const { return laser_rayleigh_range_m_; }
  inline double GetRayleighRangeOffset_m() const { return laser_rayleigh_range_offset_m_; }

 protected:
  libra::Vector<3> emitting_direction_c_{0.0};                 //!< Laser emitting direction vector @ component frame
  double emission_angle_rad_ = 0.0;                            //!< Laser emitting angle from the emitting direction [rad]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from body to component frame

  double laser_beam_waist_mm_ = 0.0;             //!< Beam waist of the laser [mm]
  double laser_rayleigh_range_m_ = 0.0;          //!< Rayleigh range of the laser [m]
  double laser_rayleigh_range_offset_m_ = 4.65;  //!< Rayleigh range position offset of the laser [m]

  // Reference
  const Dynamics* dynamics_;

  // Functions
  void Initialize(const std::string file_name, const size_t id = 0) {
    IniAccess ini_file(file_name);
    std::string name = "LASER_EMITTER_";
    const std::string section_name = name + std::to_string(static_cast<long long>(id));

    libra::Quaternion quaternion_b2c;
    ini_file.ReadQuaternion(section_name.c_str(), "quaternion_b2c", quaternion_b2c);
    libra::Vector<3> position_b_m;
    ini_file.ReadVector(section_name.c_str(), "position_b_m", position_b_m);
    dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b_m, quaternion_b2c.Conjugate()).QuaternionConjugate();

    ini_file.ReadVector(section_name.c_str(), "emitting_direction_c", emitting_direction_c_);
    emission_angle_rad_ = ini_file.ReadDouble(section_name.c_str(), "emission_angle_rad");

    laser_beam_waist_mm_ = ini_file.ReadDouble(section_name.c_str(), "laser_beam_waist_mm");
    laser_rayleigh_range_m_ = ini_file.ReadDouble(section_name.c_str(), "laser_rayleigh_range_m");
    laser_rayleigh_range_offset_m_ = ini_file.ReadDouble(section_name.c_str(), "laser_rayleigh_range_offset_m");
  }
};

#endif  // S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
