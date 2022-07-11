#pragma once

#include "../Spacecraft/FfSat.hpp"
#include "RelativeInformation.h"
#include "SimulationCase.h"

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
