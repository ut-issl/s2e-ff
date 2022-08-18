#pragma once

#include "DualQuaternion.hpp"

namespace libra {

/**
 * @class RotationFirstDualQuaternion
 * @brief Frame conversion sequence: Rotation -> Translation
 */
class RotationFirstDualQuaternion : public DualQuaternion {
 public:
  // Constructors
  /**
   * @fn Default Constructor
   * @brief make unit dual quaternion without any conversion
   */
  RotationFirstDualQuaternion();

  /**
   * @fn Constructor
   * @brief Copy dual quaternion
   */
  RotationFirstDualQuaternion(const DualQuaternion dq);

  /**
   * @fn Constructor
   * @brief Make from rotation quaternion and translation vector
   *        Frame conversion sequence: Rotation -> Translation
   * @param[in] q_rot: Quaternion for rotation. This quaternion is used after normalized in this function.
   * @param[in] v_translation: Vector for translation
   */
  RotationFirstDualQuaternion(const Quaternion q_rot, const Vector<3> v_translation);

  // Operator for a single dual quaternon
  /**
   * @fn Normalize rotation quaternion
   * @brief This function doesn't change the original DualQuaternion value
   */
  RotationFirstDualQuaternion CalcNormalizedRotationQauternion() const;

  /**
   * @fn Normalize rotation quaternion
   * @brief This function changes the original DualQuaternion value
   */
  void NormalizeRotationQauternion();

  /**
   * @fn Differential equation of dual quaternion
   * @param[in]  omega: Angular velocity [rad/s]
   * @param[in]  velocity: Velocity [-]
   * @param[out] return: Differential of dual equation
   */
  RotationFirstDualQuaternion Differential(const Vector<3>& omega, const Vector<3>& velocity) const;

  /**
   * @fn Differential equation of dual quaternion
   * @param[in]  omega: Angular velocity [rad/s]
   * @param[in]  velocity: Velocity [-]
   * @param[in]  dt: Differential time [s]
   * @param[out] return: Integrated dual equation
   */
  RotationFirstDualQuaternion Integrate(const Vector<3>& omega, const Vector<3>& velocity, const double dt) const;

  // Getter
  Vector<3> GetTranslationVector() const;

 private:
};

// Operation functions
/**
 * @fn Screw Linear Interpolation
 * @param[in]  dq1: First dual quaternion
 * @param[in]  dq2: Second dual quaternion
 * @param[in]  tau [0, 1]
 * @param[out] return: Interpolated dual equation
 * note return dq1 when the error happened
 */
RotationFirstDualQuaternion Sclerp(const RotationFirstDualQuaternion dq1, const RotationFirstDualQuaternion dq2, const double tau);

}  // namespace libra
