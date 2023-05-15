#pragma once

#include <environment/global/global_environment.hpp>
#include <environment/local/local_environment.hpp>
#include <simulation/spacecraft/installed_components.hpp>

// include for components
#include <components/ideal/force_generator.hpp>
#include <components/real/cdh/on_board_computer.hpp>

#include "../../components/aocs/InitializeRelativePositionSensor.hpp"
#include "../../components/aocs/InitializeRelativeVelocitySensor.hpp"
#include "../../components/aocs/initialize_relative_distance_sensor.hpp"
#include "../../components/ideal/InitializeRelativeAttitudeController.hpp"

class FfComponents : public InstalledComponents {
 public:
  FfComponents(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
               const SimulationConfiguration* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info);
  ~FfComponents();
  libra::Vector<3> GenerateForce_N_b();
  libra::Vector<3> GenerateTorque_Nm_b();
  void LogSetup(Logger& logger);

 private:
  // Components
  OnBoardComputer* obc_;
  RelativeDistanceSensor* relative_distance_sensor_;
  RelativePositionSensor* relative_position_sensor_;
  RelativeVelocitySensor* relative_velocity_sensor_;
  ForceGenerator* force_generator_;
  RelativeAttitudeController* relative_attitude_controller_;

  // References
  const Dynamics* dynamics_;
  const Structure* structure_;
  const LocalEnvironment* local_env_;
  const GlobalEnvironment* glo_env_;
  const SimulationConfiguration* config_;
  const RelativeInformation* rel_info_;
};
