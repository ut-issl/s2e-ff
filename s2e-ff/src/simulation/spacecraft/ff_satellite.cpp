#include "ff_satellite.hpp"

FfSat::FfSat(SimulationConfiguration* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* relative_information, const int sat_id)
    : Spacecraft(sim_config, glo_env, sat_id, relative_information) {
  if (sat_id == 0) {
    components_ = new FfComponents(dynamics_, structure_, local_environment_, glo_env, sim_config, &clock_generator_, relative_information);
  } else if (sat_id == 1) {
    components_ = new FfComponents2(dynamics_, structure_, local_environment_, glo_env, sim_config, &clock_generator_, relative_information);
  } else {
    // make FfComponents3 if you need
    components_ = new FfComponents2(dynamics_, structure_, local_environment_, glo_env, sim_config, &clock_generator_, relative_information);
  }
}
