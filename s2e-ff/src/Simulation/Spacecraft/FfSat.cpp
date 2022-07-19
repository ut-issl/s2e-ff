#include "FfSat.hpp"

FfSat::FfSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* relative_information, const int sat_id)
    : Spacecraft(sim_config, glo_env, relative_information, sat_id) {
  components_ = new FfComponents(dynamics_, structure_, local_env_, glo_env, sim_config, &clock_gen_, relative_information);
}
