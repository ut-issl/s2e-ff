#include "DualQuaternion.hpp"

namespace libra {

// Constructors
DualQuaternion::DualQuaternion() {}

DualQuaternion::DualQuaternion(const Quaternion q_real, const Quaternion q_dual) {
  q_real_ = q_real;
  q_dual_ = q_dual;
}

DualQuaternion::DualQuaternion(const double q_real_x, const double q_real_y, const double q_real_z, const double q_real_w, const double q_dual_x,
                               const double q_dual_y, const double q_dual_z, const double q_dual_w) {
  q_real_ = Quaternion(q_real_x, q_real_y, q_real_z, q_real_w);
  q_dual_ = Quaternion(q_dual_x, q_dual_y, q_dual_z, q_dual_w);
}

DualQuaternion::DualQuaternion(const Quaternion q_rot, const Vector<3> v_translation)
{
  q_real_ = q_rot;
  q_dual_ = 0.5 * q_rot * v_translation;
}


// Calculations
DualQuaternion DualQuaternion::DualNumberConjugate() const {
  Quaternion q_real_out = q_real_;
  Quaternion q_dual_out = -1.0 * q_dual_;
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion DualQuaternion::QuaternionConjugate() const {
  Quaternion q_real_out = q_real_.conjugate();
  Quaternion q_dual_out = q_dual_.conjugate();
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion DualQuaternion::DualQuaternionConjugate() const {
  Quaternion q_real_out = q_real_.conjugate();
  Quaternion q_dual_out = -1.0 * q_dual_.conjugate();
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

Vector<3> DualQuaternion::ConvertFrame(const Vector<3>& v) const
{
  DualQuaternion dq_v(0.0, 0.0, 0.0, 1.0, v[0], v[1], v[2], 0.0);
  DualQuaternion dq_out = ((*this) * dq_v) * this->DualQuaternionConjugate();

  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = dq_out.GetDualPart()[i];
  return v_out;
}

// Getters
Vector<3> DualQuaternion::GetTranslationVector() const {
  Quaternion q_out = 2.0 * q_dual_ * q_real_.conjugate();
  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = q_out[i];
  return v_out;
}


// Operation functions
DualQuaternion operator+(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs) {
  Quaternion q_real_out = dq_lhs.GetRealPart() + dq_rhs.GetRealPart();
  Quaternion q_dual_out = dq_lhs.GetDualPart() + dq_rhs.GetDualPart();
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion operator-(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs) {
  DualQuaternion dq_rhs_negative = -1.0 * dq_rhs;
  DualQuaternion dq_out = dq_lhs + dq_rhs_negative;
  return dq_out;
}

DualQuaternion operator*(const double& scalar, const DualQuaternion& dq)
{
  Quaternion q_real_out = scalar * dq.GetRealPart();
  Quaternion q_dual_out = scalar * dq.GetDualPart();
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion operator*(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs)
{
  Quaternion q_real_out = dq_lhs.GetRealPart() * dq_rhs.GetRealPart();
  Quaternion q_dual_out = (dq_lhs.GetRealPart() * dq_rhs.GetDualPart()) + (dq_rhs.GetRealPart() * dq_lhs.GetDualPart());
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

}  // namespace libra
