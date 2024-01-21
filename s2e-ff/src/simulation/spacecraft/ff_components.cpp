#include "ff_components.hpp"

#include <components/ideal/force_generator.hpp>
#include <library/initialize/initialize_file_access.hpp>

FfComponents::FfComponents(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                           const SimulationConfiguration* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info,
                           FfInterSpacecraftCommunication& inter_spacecraft_communication)
    : dynamics_(dynamics),
      structure_(structure),
      local_env_(local_env),
      glo_env_(glo_env),
      config_(config),
      rel_info_(rel_info),
      inter_spacecraft_communication_(inter_spacecraft_communication) {
  // General
  const int sat_id = 0;
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

  const std::string rel_att_file = sat_file.ReadString(section_name.c_str(), "relative_attitude_sensor_file");
  relative_attitude_sensor_ =
      new RelativeAttitudeSensor(InitializeRelativeAttitudeSensor(clock_gen, rel_att_file, compo_step_sec, *rel_info_, sat_id));

  const std::string rel_vel_file = sat_file.ReadString(section_name.c_str(), "relative_velocity_sensor_file");
  relative_velocity_sensor_ =
      new RelativeVelocitySensor(InitializeRelativeVelocitySensor(clock_gen, rel_vel_file, compo_step_sec, *rel_info_, *dynamics_, sat_id));

  const std::string ldm_file = sat_file.ReadString(section_name.c_str(), "Laser_distance_meter_file");
  config_->main_logger_->CopyFileToLogDirectory(ldm_file);
  IniAccess laser_distance_meter_file(ldm_file);
  size_t number_of_laser_distance_meters = laser_distance_meter_file.ReadInt("GENERAL", "number_of_laser_distance_meters");
  for (size_t id = 0; id < number_of_laser_distance_meters; id++) {
    laser_distance_meters_.push_back(new LaserDistanceMeter(1, clock_gen, ldm_file, *dynamics_, inter_spacecraft_communication_, id));
  }

  const std::string lm_file = sat_file.ReadString("COMPONENT_FILES", "laser_emitter_file");
  config_->main_logger_->CopyFileToLogDirectory(lm_file);
  IniAccess laser_emitter_file(lm_file);
  size_t number_of_laser_emitters = laser_emitter_file.ReadInt("GENERAL", "number_of_laser_emitters");
  for (size_t id = 0; id < number_of_laser_emitters; id++) {
    laser_emitters_.push_back(new LaserEmitter(InitializeLaserEmitter(1, clock_gen, lm_file, *dynamics_, id)));
  }
  inter_spacecraft_communication.SetLaserEmitter(laser_emitters_);

  const std::string force_generator_file = sat_file.ReadString(section_name.c_str(), "force_generator_file");
  force_generator_ = new ForceGenerator(InitializeForceGenerator(clock_gen, force_generator_file, dynamics_));

  const std::string torque_generator_file = sat_file.ReadString(section_name.c_str(), "torque_generator_file");
  torque_generator_ = new TorqueGenerator(InitializeTorqueGenerator(clock_gen, torque_generator_file, dynamics_));

  // const std::string relative_attitude_controller_file = sat_file.ReadString(section_name.c_str(), "relative_attitude_controller_file");
  // relative_attitude_controller_ = new RelativeAttitudeController(InitializeRelativeAttitudeController(
  //     clock_gen, relative_attitude_controller_file, *rel_info_, local_env_->GetCelestialInformation(), *dynamics_, sat_id));

  relative_position_attitude_observer_ = new RelativePositionAttitudeObserver(1, clock_gen, laser_distance_meters_, inter_spacecraft_communication_);

  // Debug for actuator output
  libra::Vector<3> force_N;
  force_N[0] = 1.0;
  force_N[1] = 0.0;
  force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);

  libra::Vector<3> torque_Nm;
  torque_Nm[0] = 0.1;
  torque_Nm[1] = 0.0;
  torque_Nm[2] = 0.0;
  // torque_generator_->SetTorque_b_Nm(torque_Nm);
}

FfComponents::~FfComponents() {
  delete relative_distance_sensor_;
  delete relative_position_sensor_;
  delete relative_attitude_sensor_;
  delete relative_velocity_sensor_;
  delete force_generator_;
  delete torque_generator_;
  // delete relative_attitude_controller_;
  for (auto laser_distance_meter_ : laser_distance_meters_) {
    delete laser_distance_meter_;
  }
  for (auto laser_emitter_ : laser_emitters_) {
    delete laser_emitter_;
  }
  delete relative_position_attitude_observer_;
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents::GenerateForce_b_N() {
  Vector<3> force_b_N_(0.0);
  force_b_N_ += force_generator_->GetGeneratedForce_b_N();
  return force_b_N_;
}

Vector<3> FfComponents::GenerateTorque_b_Nm() {
  Vector<3> torque_b_Nm_(0.0);
  torque_b_Nm_ += torque_generator_->GetGeneratedTorque_b_Nm();
  return torque_b_Nm_;
}

void FfComponents::LogSetup(Logger& logger) {
  logger.AddLogList(relative_distance_sensor_);
  logger.AddLogList(relative_position_sensor_);
  logger.AddLogList(relative_attitude_sensor_);
  logger.AddLogList(relative_velocity_sensor_);
  logger.AddLogList(force_generator_);
  logger.AddLogList(torque_generator_);
  for (auto laser_distance_meter_ : laser_distance_meters_) {
    logger.AddLogList(laser_distance_meter_);
  }
  logger.AddLogList(relative_position_attitude_observer_);
}
