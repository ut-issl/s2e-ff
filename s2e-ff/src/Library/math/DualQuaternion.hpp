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


  // Frame conversion
  /**
   * @fn Frame conversion of a three dimensional vector
   * @param[in]  v: Vector
   * @param[out] return: Converted vector
   */
  Vector<3> ConvertFrame(const Vector<3>& v) const;

  /**
   * @fn Frame inverse conversion of a three dimensional vector
   * @param[in]  v: Vector
   * @param[out] return: Converted vector
   */
  Vector<3> InverseConvertFrame(const Vector<3>& v) const;

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

}  // namespace libra
