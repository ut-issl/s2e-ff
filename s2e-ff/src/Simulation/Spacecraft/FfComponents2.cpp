#include "FfComponents2.hpp"

#include <Interface/InitInput/IniAccess.h>

#include <Component/IdealComponents/InitializeForceGenerator.hpp>

#include "../../Components/AOCS/InitializeRelativeDistanceSensor.hpp"
#include "../../Components/AOCS/InitializeRelativePositionSensor.hpp"
#include "../../Components/IdealComponents/InitializeRelativeAttitudeController.hpp"

FfComponents2::FfComponents2(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env,
                             const GlobalEnvironment* glo_env, const SimulationConfig* config, ClockGenerator* clock_gen,
                             const RelativeInformation* rel_info)
    : dynamics_(dynamics), structure_(structure), local_env_(local_env), glo_env_(glo_env), config_(config), rel_info_(rel_info) {
  // General
  IniAccess sat_file = IniAccess(config->sat_file_[0]);

  // Component Instantiation
  obc_ = new OBC(clock_gen);

  // Debug for actuator output
  libra::Vector<3> force_N;
  force_N[0] = 1.0;
  force_N[1] = 0.0;
  force_N[2] = 0.0;
  // force_generator_->SetForce_b_N(force_N);
}

FfComponents2::~FfComponents2() {
  // OBC must be deleted the last since it has com ports
  delete obc_;
}

Vector<3> FfComponents2::GenerateForce_N_b() {
  Vector<3> force_N_b_(0.0);
  return force_N_b_;
}

Vector<3> FfComponents2::GenerateTorque_Nm_b() {
  // No attitude control component
  Vector<3> torque_Nm_b_(0.0);
  return torque_Nm_b_;
}

void FfComponents2::LogSetup(Logger& logger) { UNUSED(logger); }
