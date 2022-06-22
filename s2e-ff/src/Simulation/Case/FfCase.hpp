#pragma once

#include "../Spacecraft/FfSat.hpp"
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
  FfSat* spacecraft_;
};
