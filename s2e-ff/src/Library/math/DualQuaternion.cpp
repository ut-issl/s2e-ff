#include "DualQuaternion.hpp"

namespace libra {

DualQuaternion::DualQuaternion() {}

DualQuaternion::DualQuaternion(const Quaternion q_real, const Quaternion q_dual) {
  q_real_ = q_real;
  q_dual_ = q_dual;
}

// Operation functions

}  // namespace libra
