#ifndef RELATIVE_ORBIT_CONTROLLER_H_
#define RELATIVE_ORBIT_CONTROLLER_H_

#include <Component/Abstract/ComponentBase.h>
#include <Interface/LogOutput/ILoggable.h>

#include "../../Library/RelativeOrbit/QuasiNonsingularRelativeOrbitalElements.hpp"
#include "../../Simulation/Spacecraft/FfComponents.hpp"

class FfComponents;

class RelativeOrbitController : public ComponentBase, public ILoggable {
 public:
  RelativeOrbitController(const int prescaler, ClockGenerator* clock_gen, FfComponents& components);
  ~RelativeOrbitController();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 protected:
  FfComponents& components_;

  // Internal variables
  double a_m_;

  QuasiNonsingularRelativeOrbitalElements target_qns_roe_with_semi_major_;  //!< Target QNS Relative OE with semi-major axis
  QuasiNonsingularRelativeOrbitalElements estimated_qns_roe_;               //!< Estimated QNS Relative OE

  double estimated_relative_distance_m_ = 0.0;  //!< Estimated relative distance

  // Constants, parameters
  double mu_m3_s2_;

  // Functions
  void EstimateStates();
};

#endif
