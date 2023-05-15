/**
 * @file ff_case.cpp
 * @brief Simulation case for FF
 */

#include "ff_case.hpp"

FfCase::FfCase(std::string ini_fname) : SimulationCase(ini_fname) { relative_information_ = RelativeInformation(); }

FfCase::~FfCase() {
  for (auto& sc : satellites_) {
    delete sc;
  }
}

void FfCase::InitializeTargetObjects() {
  // Instantiate the target of the simulation
  for (unsigned int sat_id = 0; sat_id < simulation_configuration_.number_of_simulated_spacecraft_; sat_id++) {
    FfSat* sc = new FfSat(&simulation_configuration_, global_environment_, &relative_information_, sat_id);
    satellites_.push_back(sc);
  }
  // Register the log output
  for (auto& sc : satellites_) {
    sc->LogSetup(*(simulation_configuration_.main_logger_));
  }
  relative_information_.LogSetup(*(simulation_configuration_.main_logger_));
}

void FfCase::UpdateTargetObjects() {
  // Spacecraft Update
  for (auto& sc : satellites_) {
    sc->Update(&(global_environment_->GetSimulationTime()));
  }
  // Relative Information
  relative_information_.Update();
}

std::string FfCase::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string FfCase::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
