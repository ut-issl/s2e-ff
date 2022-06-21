#pragma once

#include "FfComponents.hpp"
#include "Spacecraft.h"

class FfSat : public Spacecraft {
 public:
  FfSat(SimulationConfig *sim_config, const GlobalEnvironment *glo_env, const int sat_id);

 private:
};
