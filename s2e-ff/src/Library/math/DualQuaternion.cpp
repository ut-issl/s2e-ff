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

// Calculations
DualQuaternion DualQuaternion::Properization() const {
  if (this->GetRealPart()[3] > 0.0) {
    return *this;
  }
  return -1.0 * (*this);
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

SkrewParameters DualQuaternion::CalcSkrewParameters() const {
  SkrewParameters out;

  Vector<3> real_vector_part;
  for (int i = 0; i < 3; i++) real_vector_part[i] = this->GetRealPart()[i];
  double real_scalar_part = this->GetRealPart()[3];

  Vector<3> dual_vector_part;
  for (int i = 0; i < 3; i++) dual_vector_part[i] = this->GetDualPart()[i];
  double dual_scalar_part = this->GetDualPart()[3];

  double norm_real_vector = norm(real_vector_part);
  if (norm_real_vector < 0.0 + DBL_MIN) {
    // We cannot define skrew parameters
    out.angle_rad_ = 0.0;
    out.axis_[0] = 0.0;
    out.axis_[1] = 0.0;
    out.axis_[2] = 0.0;
    out.pitch_ = 0.0;
    out.moment_[0] = 0.0;
    out.moment_[1] = 0.0;
    out.moment_[2] = 0.0;
    return out;
  }

  // rotation axis
  for (int i = 0; i < 3; i++) out.axis_[i] = real_vector_part[i];
  normalize(out.axis_);

  // rotation angle
  out.angle_rad_ = 2.0 * atan2(norm_real_vector, real_scalar_part);

  // pitch
  out.pitch_ = -2.0 * dual_scalar_part / norm_real_vector;

  // moment
  out.moment_ = (dual_vector_part - 0.5 * out.pitch_ * real_scalar_part * out.axis_);
  out.moment_ /= norm_real_vector;

  return out;
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

}  // namespace libra
