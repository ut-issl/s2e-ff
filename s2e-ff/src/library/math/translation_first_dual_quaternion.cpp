#include "translation_first_dual_quaternion.hpp"

#include <float.h>

#include <cmath>

namespace libra {

TranslationFirstDualQuaternion::TranslationFirstDualQuaternion() {}

TranslationFirstDualQuaternion::TranslationFirstDualQuaternion(const DualQuaternion dq) : DualQuaternion(dq) {}

TranslationFirstDualQuaternion::TranslationFirstDualQuaternion(const Vector<3> v_translation, const Quaternion q_rot) {
  q_real_ = q_rot;
  q_real_.Normalize();

  // TODO: Make vector * quaternion function in core's Quaternion class
  Quaternion q_v(v_translation[0], v_translation[1], v_translation[2], 0.0);
  q_dual_ = 0.5 * (q_real_ * q_v);
}

TranslationFirstDualQuaternion TranslationFirstDualQuaternion::CalcNormalizedRotationQauternion() const {
  Quaternion q_rot = q_real_;
  Vector<3> v_translation = this->GetTranslationVector();
  q_rot.Normalize();

  TranslationFirstDualQuaternion dq_out(v_translation, q_rot);
  return dq_out;
}

void TranslationFirstDualQuaternion::NormalizeRotationQauternion() {
  Vector<3> v_translation = this->GetTranslationVector();
  q_real_.Normalize();

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
  Quaternion q_out = 2.0 * q_real_.Conjugate() * q_dual_;
  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = q_out[i];
  return v_out;
}

TranslationFirstDualQuaternion Sclerp(const TranslationFirstDualQuaternion dq1, const TranslationFirstDualQuaternion dq2, const double tau) {
  if (tau < 0.0) return dq1;
  if (tau > 1.0) return dq2;

  TranslationFirstDualQuaternion dq1_inv = dq1.Inverse();
  TranslationFirstDualQuaternion dq12 = dq1_inv * dq2;
  dq12 = dq12.Properization();

  // Calc power of dq12
  DualQuaternion dq12_tau = dq12.Power(tau);

  // When theta = 0
  if ((1.0 - fabs(dq12_tau.GetRealPart()[3])) < 0.0 + DBL_MIN) {
    // Linear interpolation of translation
    Vector<3> v_out = tau * dq1.GetTranslationVector() + (1.0 - tau) * dq2.GetTranslationVector();
    TranslationFirstDualQuaternion dq_out(v_out, dq1.GetRealPart());
    return dq_out;
  }

  // Calc interpolated dual quaternion
  TranslationFirstDualQuaternion dq_out = dq1 * dq12_tau;
  return dq_out;
}

}  // namespace libra
