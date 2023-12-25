/**
 * @file ff_inter_spacecraft_communication.h
 * @brief Base class of inter satellite communication
 */

#ifndef S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
#define S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_

// #include <simulation/multi_spacecraft/inter_spacecraft_communication.hpp>
#include "../../components/aocs/corner_cube_reflector.hpp"

/**
 * @class InterSpacecraftCommunication
 * @brief Base class of inter satellite communication
 */
class FfInterSpacecraftCommunication {
 public:
  /**
   * @fn InterSpacecraftCommunication
   * @brief Constructor
   */
  FfInterSpacecraftCommunication() {}
  /**
   * @fn ~InterSpacecraftCommunication
   * @brief Destructor
   */
  ~FfInterSpacecraftCommunication() {}

  inline void SetCornerCubeReflector(CornerCubeReflector* corner_cube_reflector) { corner_cube_reflector_ = corner_cube_reflector; }
  inline CornerCubeReflector& GetCornerCubeReflector() const { return *corner_cube_reflector_; }

 private:
  CornerCubeReflector* corner_cube_reflector_;
};

#endif  // S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
