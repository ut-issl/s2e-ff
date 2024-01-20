#include "ff_components_2.hpp"

#include <components/ideal/force_generator.hpp>
#include <library/initialize/initialize_file_access.hpp>

FfComponents2::FfComponents2(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env,
                             const GlobalEnvironment* glo_env, const SimulationConfiguration* config, ClockGenerator* clock_gen,
                             const RelativeInformation* rel_info, FfInterSpacecraftCommunication& inter_spacecraft_communication)
    : dynamics_(dynamics),
      structure_(structure),
      local_env_(local_env),
      glo_env_(glo_env),
      config_(config),
      rel_info_(rel_info),
      inter_spacecraft_communication_(inter_spacecraft_communication) {
  // General
  const size_t sat_id = 1;
  IniAccess sat_file = IniAccess(config->spacecraft_file_list_[sat_id]);

  // Component Instantiation
  obc_ = new OnBoardComputer(clock_gen);
  double compo_step_sec = glo_env_->GetSimulationTime().GetComponentStepTime_s();

  std::string file_name = sat_file.ReadString("COMPONENT_FILES", "corner_cube_reflector_file");
  config_->main_logger_->CopyFileToLogDirectory(file_name);
  IniAccess corner_cube_file(file_name);
  size_t number_of_reflectors = corner_cube_file.ReadInt("GENERAL", "number_of_reflectors");
  for (size_t id = 0; id < number_of_reflectors; id++) {
    corner_cube_reflectors_.push_back(new CornerCubeReflector(1, clock_gen, file_name, dynamics_, id));
  }
  inter_spacecraft_communication.SetCornerCubeReflector(corner_cube_reflectors_);

  const std::string qpd_file = sat_file.ReadString("COMPONENT_FILES", "qpd_positioning_sensor_file");
  config_->main_logger_->CopyFileToLogDirectory(qpd_file);
  IniAccess qpd_positioning_sensor_file(qpd_file);
  size_t number_of_qpd_positioning_sensors = qpd_positioning_sensor_file.ReadInt("GENERAL", "number_of_qpd_positioning_sensors");
  for (size_t id = 0; id < number_of_qpd_positioning_sensors; id++) {
    qpd_positioning_sensors_.push_back(new QpdPositioningSensor(
        InitializeQpdPositioningSensor(clock_gen, qpd_file, compo_step_sec, *dynamics_, inter_spacecraft_communication, id)));
  }

  information_parser_ = new InformationParser(1, clock_gen, qpd_positioning_sensors_, inter_spacecraft_communication);

  // Debug for actuator output
  libra::Vector<3> force_N;
  force_N[0] = 1.0;
  force_N[1] = 0.0;
  force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);
}

FfComponents2::~FfComponents2() {
  for (auto corner_cube_reflector : corner_cube_reflectors_) {
    delete corner_cube_reflector;
  }
  for (auto qpd_positioning_sensor : qpd_positioning_sensors_) {
    delete qpd_positioning_sensor;
  }
  delete information_parser_;
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents2::GenerateForce_b_N() {
  Vector<3> force_b_N_(0.0);
  return force_b_N_;
}

Vector<3> FfComponents2::GenerateTorque_b_Nm() {
  // No attitude control component
  Vector<3> torque_b_Nm_(0.0);
  return torque_b_Nm_;
}

void FfComponents2::LogSetup(Logger& logger) {
  for (auto qpd_positioning_sensor : qpd_positioning_sensors_) {
    logger.AddLogList(qpd_positioning_sensor);
  }
}
