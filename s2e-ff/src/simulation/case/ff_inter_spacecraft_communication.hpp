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

  inline void SetCornerCubeReflector(std::vector<CornerCubeReflector*> corner_cube_reflectors) { corner_cube_reflectors_ = corner_cube_reflectors; }
  inline CornerCubeReflector& GetCornerCubeReflector(const size_t id) const { return *corner_cube_reflectors_[id]; }
  inline size_t GetNumberOfReflectors() { return corner_cube_reflectors_.size(); }

 private:
  std::vector<CornerCubeReflector*> corner_cube_reflectors_;
};

#endif  // S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
