#include "FfComponents.hpp"

#include <Interface/InitInput/IniAccess.h>

#include <Component/IdealComponents/InitializeForceGenerator.hpp>

#include "../../Components/AOCS/InitializeRelativeDistanceSensor.hpp"
#include "../../Components/AOCS/InitializeRelativePositionSensor.hpp"
#include "../../Components/AOCS/InitializeRelativeVelocitySensor.hpp"
#include "../../Components/IdealComponents/InitializeRelativeAttitudeController.hpp"

FfComponents::FfComponents(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                           const SimulationConfig* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info)
    : dynamics_(dynamics), structure_(structure), local_env_(local_env), glo_env_(glo_env), config_(config), rel_info_(rel_info) {
  // General
  const int sat_id = 0;
  IniAccess sat_file = IniAccess(config->sat_file_[sat_id]);
  double compo_step_sec = glo_env_->GetSimTime().GetCompoStepSec();

  // Component Instantiation
  obc_ = new OBC(clock_gen);

  const std::string rel_dist_file = sat_file.ReadString("COMPONENTS_FILE", "relative_distance_sensor_file");
  relative_distance_sensor_ =
      new RelativeDistanceSensor(InitializeRelativeDistanceSensor(clock_gen, rel_dist_file, compo_step_sec, *rel_info_, sat_id));

  const std::string rel_pos_file = sat_file.ReadString("COMPONENTS_FILE", "relative_position_sensor_file");
  relative_position_sensor_ =
      new RelativePositionSensor(InitializeRelativePositionSensor(clock_gen, rel_pos_file, compo_step_sec, *rel_info_, *dynamics_, sat_id));

  const std::string rel_vel_file = sat_file.ReadString("COMPONENTS_FILE", "relative_velocity_sensor_file");
  relative_velocity_sensor_ =
      new RelativeVelocitySensor(InitializeRelativeVelocitySensor(clock_gen, rel_vel_file, compo_step_sec, *rel_info_, *dynamics_, sat_id));

  const std::string force_generator_file = sat_file.ReadString("COMPONENTS_FILE", "force_generator_file");
  force_generator_ = new ForceGenerator(InitializeForceGenerator(clock_gen, force_generator_file, dynamics_));

  relative_orbit_analyzer_ = new RelativeOrbitAnalyzer(1, clock_gen, *rel_info_);

  relative_orbit_controller_ = new RelativeOrbitControllerChief(1, clock_gen, *this);

  // Debug for actuator output
  libra::Vector<3> force_N;
  force_N[0] = 1.0;
  force_N[1] = 0.0;
  force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);
}

FfComponents::~FfComponents() {
  delete relative_distance_sensor_;
  delete relative_position_sensor_;
  delete relative_velocity_sensor_;
  delete force_generator_;
  delete relative_orbit_analyzer_;
  delete relative_orbit_controller_;
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents::GenerateForce_N_b() {
  Vector<3> force_N_b_(0.0);
  return force_N_b_;
}

Vector<3> FfComponents::GenerateTorque_Nm_b() {
  // No attitude control component
  Vector<3> torque_Nm_b_(0.0);
  return torque_Nm_b_;
}

void FfComponents::LogSetup(Logger& logger) {
  logger.AddLoggable(relative_distance_sensor_);
  logger.AddLoggable(relative_position_sensor_);
  logger.AddLoggable(relative_velocity_sensor_);
  logger.AddLoggable(relative_orbit_analyzer_);
  logger.AddLoggable(force_generator_);
}
