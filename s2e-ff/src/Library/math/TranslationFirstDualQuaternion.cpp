#include "TranslationFirstDualQuaternion.hpp"

#include <float.h>

namespace libra {

TranslationFirstDualQuaternion::TranslationFirstDualQuaternion() {}

TranslationFirstDualQuaternion::TranslationFirstDualQuaternion(const DualQuaternion dq) : DualQuaternion(dq) {}

TranslationFirstDualQuaternion::TranslationFirstDualQuaternion(const Vector<3> v_translation, const Quaternion q_rot) {
  q_real_ = q_rot;
  q_real_.normalize();

  // TODO: Make vector * quaternion function in core's Quaternion class
  Quaternion q_v(v_translation[0], v_translation[1], v_translation[2], 0.0);
  q_dual_ = 0.5 * (q_real_ * q_v);
}

TranslationFirstDualQuaternion TranslationFirstDualQuaternion::CalcNormalizedRotationQauternion() const {
  Quaternion q_rot = q_real_;
  Vector<3> v_translation = this->GetTranslationVector();
  q_rot.normalize();

  TranslationFirstDualQuaternion dq_out(v_translation, q_rot);
  return dq_out;
}

void TranslationFirstDualQuaternion::NormalizeRotationQauternion() {
  Vector<3> v_translation = this->GetTranslationVector();
  q_real_.normalize();

  TranslationFirstDualQuaternion dq_out(v_translation, q_real_);
  q_dual_ = dq_out.GetDualPart();
}

TranslationFirstDualQuaternion TranslationFirstDualQuaternion::Differential(const Vector<3>& omega, const Vector<3>& velocity) const {
  Quaternion q_omega(omega[0], omega[1], omega[2], 0.0);
  Quaternion q_velocity(velocity[0], velocity[1], velocity[2], 0.0);
  Vector<3> v_translation = this->GetTranslationVector();
  Quaternion q_translation(v_translation[0], v_translation[1], v_translation[2], 0.0);

  Quaternion q_real_out = 0.5 * q_omega * q_real_;
  Quaternion q_dual_out = 0.5 * ((q_real_out * q_translation) + (q_real_ * q_velocity));

  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

TranslationFirstDualQuaternion TranslationFirstDualQuaternion::Integrate(const Vector<3>& omega, const Vector<3>& velocity, const double dt) const {
  TranslationFirstDualQuaternion diff_dq = this->Differential(omega, velocity);
  TranslationFirstDualQuaternion dq_out = (*this) + dt * diff_dq;
  dq_out.NormalizeRotationQauternion();
  return dq_out;
}

Vector<3> TranslationFirstDualQuaternion::GetTranslationVector() const {
  Quaternion q_out = 2.0 * q_real_.conjugate() * q_dual_;
  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = q_out[i];
  return v_out;
}

TranslationFirstDualQuaternion Sclerp(const TranslationFirstDualQuaternion dq1, const TranslationFirstDualQuaternion dq2, const double tau) {
  if (tau < 0.0) return dq1;
  if (tau > 1.0) return dq2;

  TranslationFirstDualQuaternion dq1_inv = dq1.Inverse();
  TranslationFirstDualQuaternion dq12 = dq1_inv * dq2;

  // Calc rotation angle and axis
  // TODO: make function in core's quaternion library
  double cos_part = dq12.GetRealPart()[3];
  Vector<3> vector_part;
  for (int i = 0; i < 3; i++) {
    vector_part[i] = dq12.GetRealPart()[i];
  }
  double sin_part = norm(vector_part);
  double theta = 2.0 * atan2(sin_part, cos_part);
  Vector<3> axis;
  if (theta < 0.0 + DBL_MIN) {
    // No rotation
    axis = dq12.GetTranslationVector();
  } else {
    for (int i = 0; i < 3; i++) axis[i] = dq12.GetRealPart()[i];
  }
  normalize(axis);

  // Calc (dq1^-1 * dq2)^tau
  double d = dot(dq12.GetTranslationVector(), axis);
  Quaternion dq12_tau_real(axis, tau * theta);
  Quaternion dq12_tau_dual;
  for (int i = 0; i < 3; i++) dq12_tau_dual[i] = cos(tau * theta * 0.5) * axis[i];
  dq12_tau_dual[3] = -sin(tau * theta * 0.5);
  dq12_tau_dual = (0.5 * tau * d) * dq12_tau_dual;
  DualQuaternion dq12_tau(dq12_tau_real, dq12_tau_dual);

  // Calc interpolated dual quaternion
  TranslationFirstDualQuaternion dq_out = dq1 * dq12_tau;
  return dq_out;
}

}  // namespace libra
