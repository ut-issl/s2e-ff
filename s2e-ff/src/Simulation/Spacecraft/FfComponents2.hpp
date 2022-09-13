#pragma once

#include <Simulation/Spacecraft/InstalledComponents.hpp>

#include "Dynamics.h"
#include "GlobalEnvironment.h"
#include "LocalEnvironment.h"
#include "Vector.hpp"

// include for components
#include "../../Components/AOCS/RelativeDistanceSensor.hpp"
#include "../../Components/AOCS/RelativePositionSensor.hpp"
#include "../../Components/IdealComponents/ForceGenerator.hpp"
#include "../../Components/IdealComponents/RelativeAttitudeController.hpp"
#include "OBC.h"

class FfComponents2 : public InstalledComponents {
 public:
  FfComponents2(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                const SimulationConfig* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info);
  ~FfComponents2();
  libra::Vector<3> GenerateForce_N_b();
  libra::Vector<3> GenerateTorque_Nm_b();
  void LogSetup(Logger& logger);

 private:
  // Components
  OBC* obc_;

  // References
  const Dynamics* dynamics_;
  const Structure* structure_;
  const LocalEnvironment* local_env_;
  const GlobalEnvironment* glo_env_;
  const SimulationConfig* config_;
  const RelativeInformation* rel_info_;
};
