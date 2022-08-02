#include "DualQuaternion.hpp"

namespace libra {

DualQuaternion::DualQuaternion() {}

DualQuaternion::DualQuaternion(const Quaternion q_real, const Quaternion q_dual) {
  q_real_ = q_real;
  q_dual_ = q_dual;
}

// Operation functions
DualQuaternion operator+(const DualQuaternion& dq_lhs, const DualQuaternion& dq_rhs) {
  Quaternion q_real_out;
  Quaternion q_dual_out;
  q_real_out = dq_lhs.GetRealPart() + dq_rhs.GetRealPart();
  q_dual_out = dq_lhs.GetDualPart() + dq_rhs.GetDualPart();

  DualQuaternion dq_out(q_real_out, q_dual_out);
  return dq_out;
}

}  // namespace libra
