#ifndef RELATIVE_ORBIT_CONTROLLER_H_
#define RELATIVE_ORBIT_CONTROLLER_H_

#include <components/base/component.hpp>
#include <library/logger/logger.hpp>

#include "../../library/relative_orbit/quasi_nonsingular_relative_orbital_elements.hpp"
#include "../../simulation/spacecraft/ff_components_2.hpp"

class FfComponents2;

class RelativeOrbitControllerDeputy : public Component, public ILoggable {
 public:
  RelativeOrbitControllerDeputy(const int prescaler, ClockGenerator* clock_gen, const int sc_id, FfComponents2& components);
  ~RelativeOrbitControllerDeputy();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 protected:
  FfComponents2& components_;

  // Internal variables
  double a_m_ = 6928000.0;
  double mass_kg_ = 10.0;
  double impulse_output_duration_sec_ = 10.0;
  double component_update_sec_ = 0.1;
  double dv_start_s_ = 1e5;                                    // TODO: Fix the value
  double dv_timing_s_ = 1e5;                                   // TODO: Fix the value
  bool enable_thruster_ = false;                               // Change true if you want to test control algorithm

  int sc_id_;                                                  // Spacecraft ID
  QuasiNonsingularRelativeOrbitalElements target_qns_roe_;     //!< Target QNS Relative OE with semi-major axis
  QuasiNonsingularRelativeOrbitalElements estimated_qns_roe_;  //!< Estimated QNS Relative OE

  double estimated_relative_distance_m_ = 0.0;                 //!< Estimated relative distance
  double estimated_inc_vec_angle_ = 0.0;                       //!< Estimated angle of inclination vector

  libra::Vector<3> dv_rtn_m_s_{0.0};
  double dv_timing_rad_ = 0.0;
  bool first_thrust_done_ = false;
  bool second_thrust_done_ = false;

  // Constants, parameters
  double mu_m3_s2_;

  // Functions
  void EstimateStates();
  libra::Vector<3> InPlaneSingleImpulse(double& maneuver_timing_rad, const QuasiNonsingularRelativeOrbitalElements diff_qns_roe);
  libra::Vector<3> OutPlaneSingleImpulse(double& maneuver_timing_rad, const QuasiNonsingularRelativeOrbitalElements diff_qns_roe);

  libra::Vector<3> DoubleImpulse_seirios(double& first_maneuver_s, double& delta_maneuver_s, QuasiNonsingularRelativeOrbitalElements diff_qns_roe);
};

#endif
