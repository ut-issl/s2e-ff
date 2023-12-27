/**
 * @file ff_case.hpp
 * @brief Simulation case for FF
 */

#ifndef S2E_FF_SIMULATION_CASE_FF_CASE_HPP_
#define S2E_FF_SIMULATION_CASE_FF_CASE_HPP_

#include <simulation/case/simulation_case.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

#include "../spacecraft/ff_satellite.hpp"

/**
 * @class FfCase
 * @brief A simulation class for FF
 */
class FfCase : public SimulationCase {
 public:
  FfCase(std::string ini_fname);
  virtual ~FfCase();

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  std::vector<FfSat*> satellites_;
  RelativeInformation relative_information_;
  FfInterSpacecraftCommunication ff_isc_;

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

#endif  // S2E_FF_SIMULATION_CASE_FF_CASE_HPP_
