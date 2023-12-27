/**
 * @file ff_satellite.hpp
 * @brief A spacecraft class for FF simulation
 */

#ifndef S2E_FF_SIMULATION_SPACECRAFT_FF_SATELLITE_HPP_
#define S2E_FF_SIMULATION_SPACECRAFT_FF_SATELLITE_HPP_

#include <simulation/spacecraft/spacecraft.hpp>

#include "../case/ff_inter_spacecraft_communication.hpp"
#include "ff_components.hpp"
#include "ff_components_2.hpp"
/**
 * @class FfSat
 * @brief A spacecraft class
 */
class FfSat : public Spacecraft {
 public:
  /**
   * @fn FfSat
   * @brief Constructor
   */
  FfSat(SimulationConfiguration *sim_config, const GlobalEnvironment *glo_env, RelativeInformation *relative_information,
        FfInterSpacecraftCommunication &inter_spacecraft_communication, const int sat_id);

 private:
  FfInterSpacecraftCommunication &inter_spacecraft_communication_;
};

#endif  // S2E_FF_SIMULATION_SPACECRAFT_FF_SATELLITE_HPP_
