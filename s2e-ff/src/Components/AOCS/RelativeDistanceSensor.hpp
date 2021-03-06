#ifndef RELATIVE_DISTANCE_SENSOR_H_
#define RELATIVE_DISTANCE_SENSOR_H_

#include <Interface/LogOutput/ILoggable.h>
#include <RelativeInformation/RelativeInformation.h>

#include <Component/Abstract/ComponentBase.h>
#include <Component/Abstract/SensorBase.h>

class RelativeDistanceSensor : public ComponentBase, public SensorBase<1>, public ILoggable {
 public:
  RelativeDistanceSensor(const int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base,
                         const int target_sat_id, const int reference_sat_id,
                         const RelativeInformation& rel_info);
  ~RelativeDistanceSensor();
  // ComponentBase
  void MainRoutine(int count) override;

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline double GetMeasuredDistance_m() const { return measured_distance_bw_ref_target_m_[0]; }

  // Setter
  void SetTargetSatId(const int target_sat_id) { target_sat_id_ = target_sat_id; }

 protected:
  int target_sat_id_;
  const int reference_sat_id_;

  libra::Vector<1> measured_distance_bw_ref_target_m_{0.0};

  const RelativeInformation& rel_info_;
};

#endif
