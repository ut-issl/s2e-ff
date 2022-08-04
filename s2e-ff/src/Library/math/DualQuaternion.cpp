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


}  // namespace libra
