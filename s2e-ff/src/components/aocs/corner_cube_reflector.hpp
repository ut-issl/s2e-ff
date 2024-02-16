/**
 * @file corner_cube_reflector.hpp
 * @brief Corner cube reflector
 */

#ifndef S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
#define S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_

#include <components/base/component.hpp>
#include <dynamics/dynamics.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/vector.hpp>

#include "../../library/math/translation_first_dual_quaternion.hpp"

/**
 * @class CornerCubeReflector
 * @brief Corner Cube Reflector
 */
class CornerCubeReflector : public Component {
 public:
  /**
   * @fn CornerCubeReflector
   * @brief Constructor
   */
  CornerCubeReflector(const int prescaler, ClockGenerator* clock_gen);
  /**
   * @fn CornerCubeReflector
   * @brief Constructor
   */
  CornerCubeReflector(const int prescaler, ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics, const size_t id = 0);
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

  void Update(int count);

  inline libra::Vector<3> GetReflectorPosition_i_m() const { return corner_cube_reflector_position_i_m_; }
  inline libra::Vector<3> GetNormalDirection_i() const { return corner_cube_reflector_normal_direction_i_; }

  inline double GetReflectableAngle_rad() const { return reflectable_angle_rad_; }

 protected:
  libra::Vector<3> normal_direction_c_{0.0};                   //!< Reflection surface normal direction vector @ component frame
  double reflectable_angle_rad_ = 0.0;                         //!< Reflectable half angle from the normal direction [rad]
  libra::TranslationFirstDualQuaternion dual_quaternion_c2b_;  //!< Dual quaternion from body to component frame

  libra::Vector<3> corner_cube_reflector_position_i_m_{0.0};
  libra::Vector<3> corner_cube_reflector_normal_direction_i_{0.0};

  int count_ = 0;

  // Reference
  const Dynamics* dynamics_;

  // Functions
  void Initialize(const std::string file_name, const size_t id = 0);
};

#endif  // S2E_COMPONENTS_CORNER_CUBE_REFLECTOR_HPP_
