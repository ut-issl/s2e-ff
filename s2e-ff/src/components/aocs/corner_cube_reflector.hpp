/**
 * @file corner_cube_reflector.hpp
 * @brief Corner cube reflector
 */

#ifndef S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
#define S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/logger/logger.hpp>
#include <library/math/vector.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"

/**
 * @class CornerCubeReflector
 * @brief Corner Cube Reflector
 */
class CornerCubeReflector : public Component, public ILoggable {
 public:
  /**
   * @fn CornerCubeReflector
   * @brief Constructor
   */
  CornerCubeReflector() : Component(1, nullptr), dynamics_(nullptr) {}
  /**
   * @fn CornerCubeReflector
   * @brief Constructor
   */
  CornerCubeReflector(const int prescaler, ClockGenerator* clock_gen, const Dynamics* dynamics);
  /**
   * @fn ~CornerCubeReflector
   * @brief Destructor
   */
  ~CornerCubeReflector() {}

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

  inline libra::Vector<3> GetReflectorPosition_i_m() const {
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b);

    libra::Vector<3> position_i_m = dual_quaternion_i2b.InverseTransformVector(libra::Vector<3>{0.0});

    // Component -> Inertial frame
    libra::TranslationFirstDualQuaternion dual_quaternion_c2i = dual_quaternion_i2b.QuaternionConjugate() * dual_quaternion_c2b_;

    return dual_quaternion_c2i.TransformVector(libra::Vector<3>{0.0});
    // return reflector_position_i_m_;
  }
  inline libra::Vector<3> GetNormalDirection_i() const {
    // Body -> Inertial frame
    libra::Vector<3> spacecraft_position_i2b_m = dynamics_->GetOrbit().GetPosition_i_m();
    libra::Quaternion spacecraft_attitude_i2b = dynamics_->GetAttitude().GetQuaternion_i2b().Conjugate();
    libra::TranslationFirstDualQuaternion dual_quaternion_i2b(-spacecraft_position_i2b_m, spacecraft_attitude_i2b);

    libra::Vector<3> position_i_m = dual_quaternion_i2b.InverseTransformVector(libra::Vector<3>{0.0});

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

  libra::Vector<3> reflector_position_i_m_{0.0};  //!< Position of reflector @ inertia frame [m]
  libra::Vector<3> normal_direction_i_{0.0};

  // Reference
  const Dynamics* dynamics_;
};

#endif  // S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
