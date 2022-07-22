#include "ForceGenerator.hpp"

// Constructor
ForceGenerator::ForceGenerator(const int prescaler, ClockGenerator* clock_gen, const Dynamics* dynamics)
    : ComponentBase(prescaler, clock_gen), dynamics_(dynamics) {}

ForceGenerator::~ForceGenerator() {}

void ForceGenerator::MainRoutine(int count) {
  UNUSED(count);

  generated_force_b_N_ = ordered_force_b_N_;
  // TODO: Add noise

  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Quaternion q_i2rtn = dynamics_->GetOrbit().CalcQuaternionI2LVLH();
  generated_force_i_N_ = q_i2b.frame_conv_inv(generated_force_b_N_);
  generated_force_rtn_N_ = q_i2rtn.frame_conv(generated_force_i_N_);
}

void ForceGenerator::SetForce_i_N(const libra::Vector<3> force_i_N) {
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  ordered_force_b_N_ = q_i2b.frame_conv(force_i_N);
}

void ForceGenerator::SetForce_rtn_N(const libra::Vector<3> force_rtn_N) {
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Quaternion q_i2rtn = dynamics_->GetOrbit().CalcQuaternionI2LVLH();

  libra::Vector<3> force_i_N = q_i2rtn.frame_conv_inv(force_rtn_N);
  ordered_force_b_N_ = q_i2b.frame_conv(force_i_N);
}

std::string ForceGenerator::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "IdealForceGenerator";
  str_tmp += WriteVector(head + "generated_force", "b", "N", 3);

  return str_tmp;
}

std::string ForceGenerator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(generated_force_b_N_);

  return str_tmp;
}
