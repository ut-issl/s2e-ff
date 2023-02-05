#ifndef RELATIVE_VELOCITY_SENSOR_H_
#define RELATIVE_VELOCITY_SENSOR_H_

#include <Component/Abstract/ComponentBase.h>
#include <Component/Abstract/SensorBase.h>
#include <Interface/LogOutput/ILoggable.h>
#include <RelativeInformation/RelativeInformation.h>

enum class RelativeVelocitySensorErrorFrame { INERTIAL, RTN };

class RelativeVelocitySensor : public ComponentBase, public SensorBase<3>, public ILoggable {
 public:
  RelativeVelocitySensor(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base, const int target_sat_id, const int reference_sat_id,
                         const RelativeVelocitySensorErrorFrame error_frame, const RelativeInformation& rel_info, const Dynamics& dynamics);
  ~RelativeVelocitySensor();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline libra::Vector<3> GetMeasuredTargetVelocity_i_m_s() const { return measured_target_velocity_i_m_s_; }
  inline libra::Vector<3> GetMeasuredTargetVelocity_rtn_m_s() const { return measured_target_velocity_rtn_m_s_; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;
  const int reference_sat_id_;
  RelativeVelocitySensorErrorFrame error_frame_;

  libra::Vector<3> measured_target_velocity_i_m_s_{0.0};
  libra::Vector<3> measured_target_velocity_rtn_m_s_{0.0};

  const RelativeInformation& rel_info_;
  const Dynamics& dynamics_;
};

#endif
