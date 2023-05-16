#include "ff_components_2.hpp"

#include <components/ideal/initialize_force_generator.hpp>
#include <library/initialize/initialize_file_access.hpp>

FfComponents2::FfComponents2(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env,
                             const GlobalEnvironment* glo_env, const SimulationConfiguration* config, ClockGenerator* clock_gen,
                             const RelativeInformation* rel_info, const int sat_id)
    : dynamics_(dynamics), structure_(structure), local_env_(local_env), glo_env_(glo_env), config_(config), rel_info_(rel_info) {
  // General
  IniAccess sat_file = IniAccess(config->spacecraft_file_list_[sat_id]);
  double compo_step_sec = glo_env_->GetSimulationTime().GetComponentStepTime_s();

  // Component Instantiation
  obc_ = new OnBoardComputer(clock_gen);

  std::string section_name = "COMPONENT_FILES";
  const std::string rel_dist_file = sat_file.ReadString(section_name.c_str(), "relative_distance_sensor_file");
  relative_distance_sensor_ =
      new RelativeDistanceSensor(InitializeRelativeDistanceSensor(clock_gen, rel_dist_file, compo_step_sec, *rel_info_, sat_id));

  const std::string rel_pos_file = sat_file.ReadString(section_name.c_str(), "relative_position_sensor_file");
  relative_position_sensor_ =
      new RelativePositionSensor(InitializeRelativePositionSensor(clock_gen, rel_pos_file, compo_step_sec, *rel_info_, *dynamics_, sat_id));

  const std::string rel_vel_file = sat_file.ReadString(section_name.c_str(), "relative_velocity_sensor_file");
  relative_velocity_sensor_ =
      new RelativeVelocitySensor(InitializeRelativeVelocitySensor(clock_gen, rel_vel_file, compo_step_sec, *rel_info_, *dynamics_, sat_id));

  const std::string force_generator_file = sat_file.ReadString(section_name.c_str(), "force_generator_file");
  force_generator_ = new ForceGenerator(InitializeForceGenerator(clock_gen, force_generator_file, dynamics_));

  // relative_orbit_controller_ = new RelativeOrbitControllerChief(1, clock_gen, *this);

  // Debug for actuator output
  libra::Vector<3> force_N;
  force_N[0] = 1.0;
  force_N[1] = 0.0;
  force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);
}

FfComponents2::~FfComponents2() {
  delete relative_distance_sensor_;
  delete relative_position_sensor_;
  delete relative_velocity_sensor_;
  delete force_generator_;
  // delete relative_attitude_controller_;
  // delete relative_orbit_controller_;
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents2::GenerateForce_b_N() {
  Vector<3> force_b_N_(0.0);
  force_b_N_ += force_generator_->GetGeneratedForce_b_N();

  return force_b_N_;
}

Vector<3> FfComponents2::GenerateTorque_b_Nm() {
  // No attitude control component
  Vector<3> torque_b_Nm_(0.0);
  return torque_b_Nm_;
}

void FfComponents2::LogSetup(Logger& logger) {
  logger.AddLogList(relative_distance_sensor_);
  logger.AddLogList(relative_position_sensor_);
  logger.AddLogList(relative_velocity_sensor_);
  // logger.AddLogList(relative_orbit_controller_);
  logger.AddLogList(force_generator_);
}
