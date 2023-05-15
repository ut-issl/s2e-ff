/**
 * @file rotation_first_dual_quaternion.cpp
 * @brief Library for rotation first dual quaternion
 */

#include "rotation_first_dual_quaternion.hpp"

#include <float.h>

namespace libra {

RotationFirstDualQuaternion::RotationFirstDualQuaternion() {}

RotationFirstDualQuaternion::RotationFirstDualQuaternion(const DualQuaternion dq) : DualQuaternion(dq) {}

RotationFirstDualQuaternion::RotationFirstDualQuaternion(const Quaternion q_rot, const Vector<3> v_translation) {
  q_real_ = q_rot;
  q_real_.Normalize();

  // TODO: Make vector * quaternion function in core's Quaternion class
  Quaternion q_v(v_translation[0], v_translation[1], v_translation[2], 0.0);
  q_dual_ = 0.5 * (q_v * q_real_);
}

RotationFirstDualQuaternion RotationFirstDualQuaternion::CalcNormalizedRotationQauternion() const {
  Quaternion q_rot = q_real_;
  Vector<3> v_translation = this->GetTranslationVector();
  q_rot.Normalize();

  RotationFirstDualQuaternion dq_out(q_rot, v_translation);
  return dq_out;
}

void RotationFirstDualQuaternion::NormalizeRotationQauternion() {
  Vector<3> v_translation = this->GetTranslationVector();
  q_real_.Normalize();

  RotationFirstDualQuaternion dq_out(q_real_, v_translation);
  q_dual_ = dq_out.GetDualPart();
}

RotationFirstDualQuaternion RotationFirstDualQuaternion::Differential(const Vector<3>& omega, const Vector<3>& velocity) const {
  Quaternion q_omega(omega[0], omega[1], omega[2], 0.0);
  Quaternion q_velocity(velocity[0], velocity[1], velocity[2], 0.0);
  Vector<3> v_translation = this->GetTranslationVector();
  Quaternion q_translation(v_translation[0], v_translation[1], v_translation[2], 0.0);

  Quaternion q_real_out = 0.5 * q_omega * q_real_;
  Quaternion q_dual_out = 0.5 * ((q_velocity * q_real_) + (q_translation * q_real_out));

  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

RotationFirstDualQuaternion RotationFirstDualQuaternion::Integrate(const Vector<3>& omega, const Vector<3>& velocity, const double dt) const {
  RotationFirstDualQuaternion diff_dq = this->Differential(omega, velocity);
  RotationFirstDualQuaternion dq_out = (*this) + dt * diff_dq;
  dq_out.NormalizeRotationQauternion();
  return dq_out;
}

Vector<3> RotationFirstDualQuaternion::GetTranslationVector() const {
  Quaternion q_out = 2.0 * q_dual_ * q_real_.Conjugate();
  Vector<3> v_out;
  for (int i = 0; i < 3; i++) v_out[i] = q_out[i];
  return v_out;
}

RotationFirstDualQuaternion Sclerp(const RotationFirstDualQuaternion dq1, const RotationFirstDualQuaternion dq2, const double tau) {
  if (tau < 0.0) return dq1;
  if (tau > 1.0) return dq2;

  RotationFirstDualQuaternion dq1_inv = dq1.Inverse();
  RotationFirstDualQuaternion dq12 = dq1_inv * dq2;
  dq12 = dq12.Properization();

  // Calc power of dq12
  DualQuaternion dq12_tau = dq12.Power(tau);

  // When theta = 0
  if ((1.0 - fabs(dq12_tau.GetRealPart()[3])) < 0.0 + DBL_MIN) {
    // Linear interpolation of translation
    Vector<3> v_out = tau * dq1.GetTranslationVector() + (1.0 - tau) * dq2.GetTranslationVector();
    RotationFirstDualQuaternion dq_out(dq1.GetRealPart(), v_out);
    return dq_out;
  }

  // Calc interpolated dual quaternion
  RotationFirstDualQuaternion dq_out = dq1 * dq12_tau;
  return dq_out;
}

}  // namespace libra
