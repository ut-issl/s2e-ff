#pragma once

#include <Simulation/Spacecraft/InstalledComponents.hpp>

#include "Dynamics.h"
#include "GlobalEnvironment.h"
#include "LocalEnvironment.h"
#include "Vector.hpp"

// include for components
#include <Component/IdealComponents/ForceGenerator.hpp>

#include "../../Components/AOCS/RelativeDistanceSensor.hpp"
#include "../../Components/AOCS/RelativePositionSensor.hpp"
#include "../../Components/AOCS/RelativeVelocitySensor.hpp"
#include "../../Components/Analyzer/RelativeOrbitAnalyzer.hpp"
#include "../../Components/Controller/RelativeOrbitControllerDeputy.hpp"
#include "../../Components/IdealComponents/RelativeAttitudeController.hpp"
#include "OBC.h"

class RelativeOrbitControllerDeputy;

class FfComponents2 : public InstalledComponents {
 public:
  FfComponents2(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                const SimulationConfig* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info);
  ~FfComponents2();
  libra::Vector<3> GenerateForce_N_b();
  libra::Vector<3> GenerateTorque_Nm_b();
  void LogSetup(Logger& logger);

  // Getter
  inline RelativeDistanceSensor& GetRelativeDistanceSensor() const { return *relative_distance_sensor_; }
  inline RelativePositionSensor& GetRelativePositionSensor() const { return *relative_position_sensor_; }
  inline RelativeVelocitySensor& GetRelativeVelocitySensor() const { return *relative_velocity_sensor_; }
  inline ForceGenerator& GetForceGenerator() const { return *force_generator_; }

 private:
  // Components
  OBC* obc_;
  RelativeDistanceSensor* relative_distance_sensor_;
  RelativePositionSensor* relative_position_sensor_;
  RelativeVelocitySensor* relative_velocity_sensor_;
  ForceGenerator* force_generator_;

  RelativeAttitudeController* relative_attitude_controller_;
  RelativeOrbitControllerDeputy* relative_orbit_controller_;

  // References
  const Dynamics* dynamics_;
  const Structure* structure_;
  const LocalEnvironment* local_env_;
  const GlobalEnvironment* glo_env_;
  const SimulationConfig* config_;
  const RelativeInformation* rel_info_;
};
