#pragma once

#include "FfComponents.hpp"
#include "FfComponents2.hpp"
#include "Spacecraft.h"

class FfSat : public Spacecraft {
 public:
  FfSat(SimulationConfig *sim_config, const GlobalEnvironment *glo_env, RelativeInformation *relative_information, const int sat_id);

 private:
};
