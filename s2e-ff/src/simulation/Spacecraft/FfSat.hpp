#pragma once

#include <simulation/spacecraft/spacecraft.hpp>

#include "FfComponents.hpp"
#include "FfComponents2.hpp"

class FfSat : public Spacecraft {
 public:
  FfSat(SimulationConfiguration *sim_config, const GlobalEnvironment *glo_env, RelativeInformation *relative_information, const int sat_id);

 private:
};
