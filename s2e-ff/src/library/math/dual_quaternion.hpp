/**
 * @file dual_quaternion.hpp
 * @brief Library for dual quaternion
 */

#ifndef S2E_LIBRARY_MATH_DUAL_QUATERNION_HPP_
#define S2E_LIBRARY_MATH_DUAL_QUATERNION_HPP_

#include <library/math/quaternion.hpp>

namespace libra {

/**
 * @class ScrewParameters
 * @brief
 */
class ScrewParameters {
 public:
  Vector<3> axis_;    //!< Screw direction axis
  double angle_rad_;  //!< Screw rotation angle [rad]
  Vector<3> moment_;  //!< Screw moment
  double pitch_;      //!< Screw pitch
};

/**
 * @class DualQuaternion
 * @brief
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

  // Operator for a single dual quaternion
  /**
   * @fn Keeping scalar part of the rotation quaternion be positive
   */
  DualQuaternion Properization() const;

  /**
   * @fn Calculate conjugated of the dual quaternion as dual number
   */
  DualQuaternion DualNumberConjugate() const;

  /**
   * @fn Calculate conjugated of the dual quaternion as quaternion
   */
  DualQuaternion QuaternionConjugate() const;

  /**
   * @fn Calculate conjugated of the dual quaternion combined dual number and quaternion conjugate
   */
  DualQuaternion DualQuaternionConjugate() const;

  /**
   * @fn Calculate inverse of the dual quaternion
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
   * @fn Calculate screw parameters from dual quaternion
   * @param[out] return: Screw parameters
   */
  ScrewParameters CalcScrewParameters() const;

  /**
   * @fn Calculate dq^(tau)
   * @param[in]  tau: Exponents
   * @param[out] return: Screw parameters
   */
  DualQuaternion Power(const double tau) const;

  // Getter
  inline Quaternion GetRealPart() const { return q_real_; }
  inline Quaternion GetDualPart() const { return q_dual_; }
  inline Quaternion GetRotationQuaternion() const { return q_real_.Conjugate(); }  // Quaternion is used as frame conversion not vector conversion

 protected:
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

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_DUAL_QUATERNION_HPP_
