#pragma once

#include <Library/math/Quaternion.hpp>

namespace libra {

/**
 * @class DualQuaternion
 * @brief Dual quaternion
 */
class DualQuaternion {
 public:
  // Constructors
  /**
   * @fn Default constructor
   * @brief TODO: decide do nothing? set default value?
   */
  DualQuaternion();

  /**
   * @fn Constructor
   * @brief Make from two quaternion
   * @param[in] q_real: Real part Quaternion
   * @param[in] q_dual: Dual part Quaternion
   */
  DualQuaternion(const Quaternion q_real, const Quaternion q_dual);

  // Operator for a single dual quaternon
  /**
   * @fn Normalize rotation quaternion?
   */
  // DualQuaternion normalize(void);

  /**
   * @fn Calulate conjugated dual quaternion
   */
  DualQuaternion Conjugate(void) const;

  // Frame conversion
  /**
   * @fn Frame conversion of a three dimensional vector
   * @param[in]  v: Vector
   * @param[out] return: Converted vector
   */
  Vector<3> ConvertFrame(const Vector<3>& v);

  /**
   * @fn Frame conversion with conjugated dual quaternion of a three dimensional vector
   * @param[in]  v: Vector
   * @param[out] return: Converted vector
   */
  Vector<3> ConvertFrameConjugate(const Vector<3>& v);

 private:
  Quaternion q_real_;  //!< Real part Quaternion
  Quaternion q_dual_;  //!< Dual part Quaternion
};

/**
 * @fn Addition of Dual Quaternion
 * @param[in] dq_lhs: Dual Quaternion left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
Quaternion operator+(const Quaternion& dq_lhs, const Quaternion& dq_rhs);

/**
 * @fn Subtraction of Dual Quaternion
 * @param[in] dq_lhs: Dual Quaternion left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
Quaternion operator-(const Quaternion& dq_lhs, const Quaternion& dq_rhs);

/**
 * @fn Multiplication of Dual Quaternion
 * @param[in] dq_lhs: Dual Quaternion left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
Quaternion operator*(const Quaternion& dq_lhs, const Quaternion& dq_rhs);

/**
 * @fn Multiplication of scalar and Dual Quaternion
 * @param[in] lhs: scalar left hand side
 * @param[in] dq_rhs: Dual Quaternion right hand side
 */
Quaternion operator*(const double& lhs, const Quaternion& dq_rhs);

}  // namespace libra
