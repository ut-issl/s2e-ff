#pragma once

#include <simulation/case/simulation_case.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

#include "../spacecraft/FfSat.hpp"

class FfCase : public SimulationCase {
 public:
  FfCase(std::string ini_fname);
  virtual ~FfCase();

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  std::vector<FfSat*> satellites_;
  RelativeInformation relative_information_;

  /**
   * @fn InitializeTargetObjects
   * @brief Override function of InitializeTargetObjects in SimulationCase
   */
  void InitializeTargetObjects();

  /**
   * @fn UpdateTargetObjects
   * @brief Override function of Main in SimulationCase
   */
  void UpdateTargetObjects();
};
