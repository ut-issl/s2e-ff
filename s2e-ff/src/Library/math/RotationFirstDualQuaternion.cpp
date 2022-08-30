#include "RotationFirstDualQuaternion.hpp"

#include <float.h>

namespace libra {

RotationFirstDualQuaternion::RotationFirstDualQuaternion() {}

RotationFirstDualQuaternion::RotationFirstDualQuaternion(const DualQuaternion dq) : DualQuaternion(dq) {}

RotationFirstDualQuaternion::RotationFirstDualQuaternion(const Quaternion q_rot, const Vector<3> v_translation) {
  q_real_ = q_rot;
  q_real_.normalize();

  // TODO: Make vector * quaternion function in core's Quaternion class
  Quaternion q_v(v_translation[0], v_translation[1], v_translation[2], 0.0);
  q_dual_ = 0.5 * (q_v * q_real_);
}

RotationFirstDualQuaternion RotationFirstDualQuaternion::CalcNormalizedRotationQauternion() const {
  Quaternion q_rot = q_real_;
  Vector<3> v_translation = this->GetTranslationVector();
  q_rot.normalize();

  RotationFirstDualQuaternion dq_out(q_rot, v_translation);
  return dq_out;
}

void RotationFirstDualQuaternion::NormalizeRotationQauternion() {
  Vector<3> v_translation = this->GetTranslationVector();
  q_real_.normalize();

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
  Quaternion q_out = 2.0 * q_dual_ * q_real_.conjugate();
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

  // Calc skrew params
  ScrewParameters screw = dq12.CalcScrewParameters();

  // When theta = 0
  if (screw.angle_rad_ < 0.0 + DBL_MIN) {
    // Linear interpolation of translation
    Vector<3> v_out = tau * dq1.GetTranslationVector() + (1.0 - tau) * dq2.GetTranslationVector();
    RotationFirstDualQuaternion dq_out(dq1.GetRealPart(), v_out);
    return dq_out;
  }

  // Calc (dq1^-1 * dq2)^tau
  Quaternion dq12_tau_real(screw.axis_, tau * screw.angle_rad_);
  Quaternion dq12_tau_dual;
  for (int i = 0; i < 3; i++) {
    dq12_tau_dual[i] =
        (0.5 * tau * screw.pitch_) * cos(tau * screw.angle_rad_ * 0.5) * screw.axis_[i] + sin(tau * screw.angle_rad_ * 0.5) * screw.moment_[i];
  }
  dq12_tau_dual[3] = -(0.5 * tau * screw.pitch_) * sin(tau * screw.angle_rad_ * 0.5);
  DualQuaternion dq12_tau(dq12_tau_real, dq12_tau_dual);

  // Calc interpolated dual quaternion
  RotationFirstDualQuaternion dq_out = dq1 * dq12_tau;
  return dq_out;
}

}  // namespace libra
