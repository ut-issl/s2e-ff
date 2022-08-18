#include "DualQuaternion.hpp"

#include <float.h>

namespace libra {

// Constructors
DualQuaternion::DualQuaternion() {
  q_real_ = Quaternion(0.0, 0.0, 0.0, 1.0);
  q_dual_ = Quaternion(0.0, 0.0, 0.0, 0.0);
}

DualQuaternion::DualQuaternion(const Quaternion q_real, const Quaternion q_dual) {
  q_real_ = q_real;
  q_dual_ = q_dual;
}

DualQuaternion::DualQuaternion(const double q_real_x, const double q_real_y, const double q_real_z, const double q_real_w, const double q_dual_x,
                               const double q_dual_y, const double q_dual_z, const double q_dual_w) {
  q_real_ = Quaternion(q_real_x, q_real_y, q_real_z, q_real_w);
  q_dual_ = Quaternion(q_dual_x, q_dual_y, q_dual_z, q_dual_w);
}

DualQuaternion::DualQuaternion(const Quaternion q_rot, const Vector<3> v_translation) {
  q_real_ = q_rot;
  q_real_.normalize();

  // TODO: Make vector * quaternion function in core's Quaternion class
  Quaternion q_v(v_translation[0], v_translation[1], v_translation[2], 0.0);
  q_dual_ = 0.5 * (q_v * q_real_);
}

DualQuaternion::DualQuaternion(const Vector<3> v_translation, const Quaternion q_rot) {
  q_real_ = q_rot;
  q_real_.normalize();

  // TODO: Make vector * quaternion function in core's Quaternion class
  Quaternion q_v(v_translation[0], v_translation[1], v_translation[2], 0.0);
  q_dual_ = 0.5 * (q_real_ * q_v);
}

// Calculations
DualQuaternion DualQuaternion::CalcNormalizedRotationQauternion() const {
  Quaternion q_rot = q_real_;
  Vector<3> v_translation = this->GetRotationFirstTranslationVector();
  q_rot.normalize();

  DualQuaternion dq_out(q_rot, v_translation);
  return dq_out;
}

void DualQuaternion::NormalizeRotationQauternion() {
  Vector<3> v_translation = this->GetRotationFirstTranslationVector();
  q_real_.normalize();

  DualQuaternion dq_out(q_real_, v_translation);
  q_dual_ = dq_out.GetDualPart();
}

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

Vector<3> DualQuaternion::TransformVector(const Vector<3>& v) const {
  DualQuaternion dq_v(0.0, 0.0, 0.0, 1.0, v[0], v[1], v[2], 0.0);
  DualQuaternion dq_out = ((*this) * dq_v) * this->DualQuaternionConjugate();

  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = dq_out.GetDualPart()[i];
  return v_out;
}

Vector<3> DualQuaternion::InverseTransformVector(const Vector<3>& v) const {
  DualQuaternion dq_v(0.0, 0.0, 0.0, 1.0, v[0], v[1], v[2], 0.0);
  DualQuaternion dq_inv = this->QuaternionConjugate();

  DualQuaternion dq_out = (dq_inv * dq_v) * dq_inv.DualQuaternionConjugate();

  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = dq_out.GetDualPart()[i];
  return v_out;
}

DualQuaternion DualQuaternion::Differential(const Vector<3>& omega, const Vector<3>& velocity) const {
  Quaternion q_omega(omega[0], omega[1], omega[2], 0.0);
  Quaternion q_velocity(velocity[0], velocity[1], velocity[2], 0.0);

  Quaternion q_real_out = 0.5 * q_omega * q_real_;
  Quaternion q_dual_out = 0.5 * ((q_velocity * q_real_) + 0.5 * (q_velocity * q_omega * q_real_));

  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion DualQuaternion::Integrate(const Vector<3>& omega, const Vector<3>& velocity, const double dt) const {
  DualQuaternion diff_dq = this->Differential(omega, velocity);
  DualQuaternion dq_out = (*this) + dt * diff_dq;
  dq_out.NormalizeRotationQauternion();
  return dq_out;
}

// Getters
Vector<3> DualQuaternion::GetRotationFirstTranslationVector() const {
  Quaternion q_out = 2.0 * q_dual_ * q_real_.conjugate();
  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = q_out[i];
  return v_out;
}

Vector<3> DualQuaternion::GetTranslationFirstTranslationVector() const {
  Quaternion q_out = 2.0 * q_real_.conjugate() * q_dual_;
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

DualQuaternion operator*(const double& scalar, const DualQuaternion& dq) {
  Quaternion q_real_out = scalar * dq.GetRealPart();
  Quaternion q_dual_out = scalar * dq.GetDualPart();
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion operator*(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs) {
  Quaternion q_real_out = dq_lhs.GetRealPart() * dq_rhs.GetRealPart();
  Quaternion q_dual_out = (dq_lhs.GetRealPart() * dq_rhs.GetDualPart()) + (dq_lhs.GetDualPart() * dq_rhs.GetRealPart());
  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

DualQuaternion Sclerp(const DualQuaternion dq1, const DualQuaternion dq2, const double tau) {
  if (tau < 0.0) return dq1;
  if (tau > 1.0) return dq1;

  DualQuaternion dq1_inv = dq1.Inverse();
  DualQuaternion dq12 = dq1_inv * dq2;

  // Calc rotation angle and axis
  // TODO: make function in core's quaternion library
  double theta = 2.0 * acos(dq12.GetRealPart()[3]);
  Vector<3> axis;
  if (theta < 0.0 + DBL_MIN) {
    // No rotation
    axis = dq12.GetRotationFirstTranslationVector();
  } else {
    for (int i = 0; i < 3; i++) axis[i] = dq12.GetRealPart()[i];
  }
  normalize(axis);

  // Calc (dq1^-1 * dq2)^tau
  double d = dot(dq12.GetRotationFirstTranslationVector(), axis);
  Quaternion dq12_tau_real(axis, tau * theta);
  Quaternion dq12_tau_dual;
  for (int i = 0; i < 3; i++) dq12_tau_dual[i] = cos(tau * theta * 0.5) * axis[i];
  dq12_tau_dual[3] = -sin(tau * theta * 0.5);
  dq12_tau_dual = (0.5 * tau * d) * dq12_tau_dual;
  DualQuaternion dq12_tau(dq12_tau_real, dq12_tau_dual);

  // Calc interpolated dual quaternion
  DualQuaternion dq_out = dq1 * dq12_tau;
  return dq_out;
}

}  // namespace libra
