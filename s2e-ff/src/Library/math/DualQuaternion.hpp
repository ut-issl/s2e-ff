#pragma once

#include <Library/math/Quaternion.hpp>

namespace libra {

/**
 * @class DualQuaternion
 * @brief Frame conversion sequence: Rotation -> Translation
 */
class DualQuaternion {
 public:
  // Constructors
  /**
   * @fn Default constructor
   * @brief make unit dual quaternion without any conversion
   */
  DualQuaternion();

  /**
   * @fn Constructor
   * @brief Make from two quaternion
   * @param[in] q_real: Real part Quaternion
   * @param[in] q_dual: Dual part Quaternion
   */
  DualQuaternion(const Quaternion q_real, const Quaternion q_dual);

  /**
   * @fn Constructor
   * @brief Make from eight numbers
   */
  DualQuaternion(const double q_real_x, const double q_real_y, const double q_real_z, const double q_real_w, const double q_dual_x,
                 const double q_dual_y, const double q_dual_z, const double q_dual_w);

  /**
   * @fn Constructor
   * @brief Make from rotation quaternion and translation vector
   *        Frame conversion sequence: Rotation -> Translation
   * @param[in] q_rot: Quaternion for rotation. This quaternion is used after normalized in this function.
   * @param[in] v_translation: Vector for translation
   */
  DualQuaternion(const Quaternion q_rot, const Vector<3> v_translation);

  // Operator for a single dual quaternon
  /**
   * @fn Normalize rotation quaternion
   * @brief This function doesn't change the original DualQuaternion value
   */
  DualQuaternion CalcNormalizedRotationQauternion() const;

  /**
   * @fn Normalize rotation quaternion
   * @brief This function changes the original DualQuaternion value
   */
  void NormalizeRotationQauternion();

  /**
   * @fn Calulate conjugated of the dual quaternion as dual number
   */
  DualQuaternion DualNumberConjugate() const;

  /**
   * @fn Calulate conjugated of the dual quaternion as quaternion
   */
  DualQuaternion QuaternionConjugate() const;

  /**
   * @fn Calulate conjugated of the dual quaternion combined dual number and quaternion conjugate
   */
  DualQuaternion DualQuaternionConjugate() const;

  /**
   * @fn Calulate inverse of the dual quaternion
   */
  inline DualQuaternion Inverse() const { return this->QuaternionConjugate(); };

  // Frame conversion
  /**
   * @fn Transform a three dimensional vector
   * @param[in]  v: Vector
   * @param[out] return: Converted vector
   */
  Vector<3> TransformVector(const Vector<3>& v) const;

  /**
   * @fn Inverse transform a three dimensional vector
   * @param[in]  v: Vector
   * @param[out] return: Converted vector
   */
  Vector<3> InverseTransformVector(const Vector<3>& v) const;

  /**
   * @fn Differential equation of dual quaternion
   * @param[in]  omega: Angular velocity [rad/s]
   * @param[in]  velocity: Velocity [-]
   * @param[out] return: Differential of dual equation
   */
  DualQuaternion Differential(const Vector<3>& omega, const Vector<3>& velocity) const;

  /**
   * @fn Differential equation of dual quaternion
   * @param[in]  omega: Angular velocity [rad/s]
   * @param[in]  velocity: Velocity [-]
   * @param[in]  dt: Differential time [s]
   * @param[out] return: Integrated dual equation
   */
  DualQuaternion Integrate(const Vector<3>& omega, const Vector<3>& velocity, const double dt) const;

  // Getter
  inline Quaternion GetRealPart() const { return q_real_; }
  inline Quaternion GetDualPart() const { return q_dual_; }
  inline Quaternion GetRotationQuaternion() const { return q_real_; }
  Vector<3> GetTranslationVector() const;

 private:
  Quaternion q_real_;  //!< Real part Quaternion
  Quaternion q_dual_;  //!< Dual part Quaternion
};

// Operation functions
/**
 * @fn Addition of Dual Quaternion
 * @param[in] dq_lhs: Dual Quaternion left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
DualQuaternion operator+(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs);

/**
 * @fn Subtraction of Dual Quaternion
 * @param[in] dq_lhs: Dual Quaternion left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
DualQuaternion operator-(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs);

/**
 * @fn Multiplication of scalar and Dual Quaternion
 * @param[in] scalar: scalar value
 * @param[in] dq: Dual Quaternion
 */
DualQuaternion operator*(const double& scalar, const DualQuaternion& dq);

/**
 * @fn Multiplication of Dual Quaternion
 * @param[in] dq_lhs: Dual Quaternion left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
DualQuaternion operator*(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs);

/**
 * @fn Screw Linear Interpolation
 * @param[in]  dq1: First dual quaternion
 * @param[in]  dq2: Second dual quaternion
 * @param[in]  tau [0, 1]
 * @param[out] return: Interpolated dual equation
 * note return dq1 when the error happened
 */
DualQuaternion Sclerp(const DualQuaternion dq1, const DualQuaternion dq2, const double tau);

}  // namespace libra
