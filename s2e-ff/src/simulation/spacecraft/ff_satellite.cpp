#include "ff_satellite.hpp"

FfSat::FfSat(SimulationConfiguration* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* relative_information,
             FfInterSpacecraftCommunication& inter_spacecraft_communication, const int sat_id)
    : Spacecraft(sim_config, glo_env, sat_id, relative_information), inter_spacecraft_communication_(inter_spacecraft_communication) {
  if (sat_id == 0) {
    components_ = new FfComponents(dynamics_, structure_, local_environment_, glo_env, sim_config, &clock_generator_, relative_information,
                                   inter_spacecraft_communication_);
  } else if (sat_id == 1) {
    components_ = new FfComponents2(dynamics_, structure_, local_environment_, glo_env, sim_config, &clock_generator_, relative_information,
                                    inter_spacecraft_communication_, sat_id);
  } else {
    components_ = new FfComponents2(dynamics_, structure_, local_environment_, glo_env, sim_config, &clock_generator_, relative_information,
                                    inter_spacecraft_communication_, sat_id);
    ;
  }
}
