#include "FfCase.hpp"

FfCase::FfCase(std::string ini_fname) : SimulationCase(ini_fname) { relative_information_ = RelativeInformation(); }

FfCase::~FfCase() {
  for (auto& sc : satellites_) {
    delete sc;
  }
}

void FfCase::Initialize() {
  // Instantiate the target of the simulation
  for (int sat_id = 0; sat_id < sim_config_.num_of_simulated_spacecraft_; sat_id++) {
    FfSat* sc = new FfSat(&sim_config_, glo_env_, &relative_information_, sat_id);
    satellites_.push_back(sc);
  }
  // Register the log output
  glo_env_->LogSetup(*(sim_config_.main_logger_));
  for (auto& sc : satellites_) {
    sc->LogSetup(*(sim_config_.main_logger_));
  }
  relative_information_.LogSetup(*(sim_config_.main_logger_));

  // Write headers to the log
  sim_config_.main_logger_->WriteHeaders();

  // Start the simulation
  std::cout << "\nSimulationDateTime \n";
  glo_env_->GetSimTime().PrintStartDateTime();
}

void FfCase::Main() {
  glo_env_->Reset();  // for MonteCarlo Sim
  while (!glo_env_->GetSimTime().GetState().finish) {
    // Logging
    if (glo_env_->GetSimTime().GetState().log_output) {
      sim_config_.main_logger_->WriteValues();
    }
    // Global Environment Update
    glo_env_->Update();
    // Spacecraft Update
    for (auto& sc : satellites_) {
      sc->Update(&(glo_env_->GetSimTime()));
    }
    // Relative Information
    relative_information_.Update();
    // Debug output
    if (glo_env_->GetSimTime().GetState().disp_output) {
      std::cout << "Progresss: " << glo_env_->GetSimTime().GetProgressionRate() << "%\r";
    }
  }
}

std::string FfCase::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string FfCase::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
