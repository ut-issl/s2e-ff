#ifndef RELATIVE_POSITION_SENSOR_H_
#define RELATIVE_POSITION_SENSOR_H_

#include <Component/Abstract/ComponentBase.h>
#include <Component/Abstract/SensorBase.h>
#include <Interface/LogOutput/ILoggable.h>
#include <RelativeInformation/RelativeInformation.h>

enum class RelativePositionSensorErrorFrame {
  INERTIAL,
  RTN,
  BODY
};

class RelativePositionSensor : public ComponentBase, public SensorBase<3>, public ILoggable {
 public:
  RelativePositionSensor(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base, const int target_sat_id, const int reference_sat_id,
                         const RelativePositionSensorErrorFrame error_frame, const RelativeInformation& rel_info, const Dynamics& dynamics);
  ~RelativePositionSensor();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline libra::Vector<3> GetMeasuredTargetPosition_i_m() const { return measured_target_position_i_m_; }
  inline libra::Vector<3> GetMeasuredTargetPosition_rtn_m() const { return measured_target_position_rtn_m_; }
  inline libra::Vector<3> GetMeasuredTargetPosition_b_m() const { return measured_target_position_body_m_; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;
  const int reference_sat_id_;
  RelativePositionSensorErrorFrame error_frame_;

  libra::Vector<3> measured_target_position_i_m_{0.0};
  libra::Vector<3> measured_target_position_rtn_m_{0.0};
  libra::Vector<3> measured_target_position_body_m_{0.0};

  const RelativeInformation& rel_info_;
  const Dynamics& dynamics_;
};

#endif
