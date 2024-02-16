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
  laser_distance_meter_ = new LaserDistanceMeter(1, clock_gen, ldm_file, *dynamics_, inter_spacecraft_communication_);

  const std::string qpd_file = sat_file.ReadString(section_name.c_str(), "qpd_positioning_sensor_file");
  qpd_positioning_sensor_ =
      new QpdPositioningSensor(InitializeQpdPositioningSensor(clock_gen, qpd_file, compo_step_sec, *dynamics_, inter_spacecraft_communication_));

  const std::string force_generator_file = sat_file.ReadString(section_name.c_str(), "force_generator_file");
  force_generator_ = new ForceGenerator(InitializeForceGenerator(clock_gen, force_generator_file, dynamics_));

  const std::string torque_generator_file = sat_file.ReadString(section_name.c_str(), "torque_generator_file");
  torque_generator_ = new TorqueGenerator(InitializeTorqueGenerator(clock_gen, torque_generator_file, dynamics_));

  relative_orbit_analyzer_ = new RelativeOrbitAnalyzer(1, clock_gen, *rel_info_);

  relative_orbit_controller_ = new RelativeOrbitControllerChief(1, clock_gen, *this);

  /*
    const std::string relative_attitude_controller_file = sat_file.ReadString(section_name.c_str(), "relative_attitude_controller_file");
    relative_attitude_controller_ = new RelativeAttitudeController(InitializeRelativeAttitudeController(
        clock_gen, relative_attitude_controller_file, *rel_info_, local_env_->GetCelestialInformation(), *dynamics_, sat_id));
  */

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
  delete relative_orbit_analyzer_;
  delete relative_orbit_controller_;
  delete torque_generator_;
  // delete relative_attitude_controller_;
  delete laser_distance_meter_;
  delete qpd_positioning_sensor_;
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents::GenerateForce_b_N() {
  Vector<3> force_b_N_(0.0);
  // force_b_N_ += force_generator_->GetGeneratedForce_b_N();
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
  logger.AddLogList(relative_orbit_analyzer_);
  logger.AddLogList(force_generator_);
  logger.AddLogList(torque_generator_);
  logger.AddLogList(laser_distance_meter_);
  logger.AddLogList(qpd_positioning_sensor_);
}
