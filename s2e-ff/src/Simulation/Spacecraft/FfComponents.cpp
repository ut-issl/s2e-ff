#include "FfComponents.hpp"

#include <Interface/InitInput/IniAccess.h>

#include "../../Components/AOCS/InitializeRelativeDistanceSensor.hpp"

FfComponents::FfComponents(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                           const SimulationConfig* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info)
    : dynamics_(dynamics), structure_(structure), local_env_(local_env), glo_env_(glo_env), config_(config), rel_info_(rel_info) {
  // General
  IniAccess sat_file = IniAccess(config->sat_file_[0]);
  double compo_step_sec = glo_env_->GetSimTime().GetCompoStepSec();

  // Component Instantiation
  obc_ = new OBC(clock_gen);

  const std::string rel_dist_file = sat_file.ReadString("COMPONENTS_FILE", "relative_distance_sensor_file");
  relative_distance_sensor_ = new RelativeDistanceSensor(InitializeRelativeDistanceSensor(clock_gen, rel_dist_file, compo_step_sec, *rel_info_));
}

FfComponents::~FfComponents() {
  delete relative_distance_sensor_;
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents::GenerateForce_N_b() {
  // There is no orbit control component, so it remains 0
  Vector<3> force_N_b_(0.0);
  return force_N_b_;
}

Vector<3> FfComponents::GenerateTorque_Nm_b() {
  // No attitude control component
  Vector<3> torque_Nm_b_(0.0);
  return torque_Nm_b_;
}

void FfComponents::LogSetup(Logger& logger) { logger.AddLoggable(relative_distance_sensor_); }
