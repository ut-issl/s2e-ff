#include "DualQuaternion.hpp"

namespace libra {

DualQuaternion::DualQuaternion() {}

DualQuaternion::DualQuaternion(const Quaternion q_rotation, const Quaternion q_translation) {
  q_rotation_ = q_rotation;
  q_translation_ = q_translation;
}


}  // namespace libra