#ifndef RELATIVE_ORBIT_ANALYZER_H_
#define RELATIVE_ORBIT_ANALYZER_H_

#include <Component/Abstract/ComponentBase.h>
#include <Interface/LogOutput/ILoggable.h>
#include <RelativeInformation/RelativeInformation.h>

#include "../../Library/RelativeOrbit/QuasiNonsingularRelativeOrbitalElements.hpp"

class RelativeOrbitAnalyzer : public ComponentBase, public ILoggable {
 public:
  RelativeOrbitAnalyzer(const int prescaler, ClockGenerator* clock_gen, const RelativeInformation& rel_info);
  ~RelativeOrbitAnalyzer();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 protected:
  const RelativeInformation& rel_info_;

  libra::Matrix<3, 3> dcm_eci_to_img_{0.0};
  // Target-1
  libra::Vector<3> d_chief_to_target1_img_{0.0};
  double d_norm_chief_to_target1_ = 0.0;
  double baseline_angle1_in_img_rad_ = 0.0;
  // Target-2
  libra::Vector<3> d_chief_to_target2_img_{0.0};
  double d_norm_chief_to_target2_ = 0.0;
  double baseline_angle2_in_img_rad_ = 0.0;

  libra::Matrix<3, 3> MakeDcmEciToImg(const double dec_deg, const double ra_deg);
};

#endif
