/**
 * @file corner_cube_reflector.hpp
 * @brief Corner cube reflector
 */

#ifndef S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
#define S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_

#include <dynamics/dynamics.hpp>
#include <library/math/vector.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"

/**
 * @class CornerCubeReflector
 * @brief Corner Cube Reflector
 */
class CornerCubeReflector {
 public:
  /**
   * @fn CornerCubeReflector
   * @brief Constructor
   */
  CornerCubeReflector() : dynamics_(nullptr) {}
  /**
   * @fn CornerCubeReflector
   * @brief Constructor
   */
  CornerCubeReflector(const Dynamics* dynamics) {
    normal_direction_c_[0] = 0.0;
    normal_direction_c_[1] = 0.0;
    normal_direction_c_[2] = 1.0;
    reflectable_angle_rad_ = 0.1;

    libra::Quaternion q_b2c(0.0, 1.0, 0.0, 1.0);
    libra::Vector<3> position_b2c_m;
    position_b2c_m[0] = 0.5;
    position_b2c_m[1] = 0.0;
    position_b2c_m[2] = 0.0;
    dual_quaternion_c2b_ = libra::TranslationFirstDualQuaternion(-position_b2c_m, q_b2c.Conjugate()).QuaternionConjugate();
  }
  /**
   * @fn ~CornerCubeReflector
   * @brief Destructor
   */
  ~CornerCubeReflector() {}

  inline libra::Vector<3> GetReflectorPosition_i_m() const {
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    return dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
  }

  inline libra::Vector<3> GetNormalDirection_i() const {
    // Body -> Inertial frame
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b.Conjugate());

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    libra::Vector<3> reflector_position_i_m = dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
    libra::Vector<3> normal_direction_i = dual_quaternion_c2i.TransformVector(normal_direction_c_);
    normal_direction_i -= reflector_position_i_m;

    return normal_direction_i;
  }

  inline double GetReflectableAngle_rad() const { return reflectable_angle_rad_; }

 protected:
  libra::Vector<3> normal_direction_c_{0.0};                   //!< Reflection surface normal direction vector @ component frame
  double reflectable_angle_rad_;                               //!< Reflectable half angle from the normal direction [rad]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from body to component frame

  // Reference
  const Dynamics* dynamics_;
};

#endif  // S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
