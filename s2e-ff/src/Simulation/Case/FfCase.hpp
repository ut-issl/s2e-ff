#pragma once

#include <simulation/case/simulation_case.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

#include "../Spacecraft/FfSat.hpp"

class FfCase : public SimulationCase {
 public:
  FfCase(std::string ini_fname);
  virtual ~FfCase();

  void Initialize();
  void Main();

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  std::vector<FfSat*> satellites_;
  RelativeInformation relative_information_;
};
