#pragma once

#include <Library/math/Quaternion.hpp>

namespace libra {

/**
 * @class DualQuaternion
 * @brief Dual quaternion
 */
class DualQuaternion {
 public:
  /**
   * @fn Default constructor
   * @brief do nothing
   */
  DualQuaternion();

  /**
   * @fn Constructor
   * @brief Make from two quaternion
   * @param q_rot: Quaternion for rotation
   * @param q_shift: Quaternion for translation
  */
  DualQuaternion(const Quaternion q_rotation, const Quaternion q_translation);

 private:
  Quaternion q_rotation_;    //!< Rotation Quaternion
  Quaternion q_translation_; //!< Translation Quaternion
};

} // namespace libra
