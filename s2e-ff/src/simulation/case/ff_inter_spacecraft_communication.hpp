/**
 * @file ff_inter_spacecraft_communication.h
 * @brief Base class of inter satellite communication
 */

#ifndef S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
#define S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_

// #include <simulation/multi_spacecraft/inter_spacecraft_communication.hpp>
#include <library/math/vector.hpp>

#include "../../components/aocs/corner_cube_reflector.hpp"
#include "../../components/aocs/laser_emitter.hpp"

/**
 * @class InterSpacecraftCommunication
 * @brief Base class of inter satellite communication
 */
class FfInterSpacecraftCommunication {
 public:
  /**
   * @fn InterSpacecraftCommunication
   * @brief Constructor
   */
  FfInterSpacecraftCommunication() {}
  /**
   * @fn ~InterSpacecraftCommunication
   * @brief Destructor
   */
  ~FfInterSpacecraftCommunication() {}

  inline void SetCornerCubeReflector(std::vector<CornerCubeReflector*> corner_cube_reflectors) { corner_cube_reflectors_ = corner_cube_reflectors; }
  inline void SetLaserEmitter(std::vector<LaserEmitter*> laser_emitters) { laser_emitters_ = laser_emitters; }
  inline CornerCubeReflector& GetCornerCubeReflector(const size_t id) const { return *(corner_cube_reflectors_[id]); }
  inline LaserEmitter& GetLaserEmitter(const size_t id) const { return *(laser_emitters_[id]); }
  inline size_t GetNumberOfReflectors() const { return corner_cube_reflectors_.size(); }
  inline size_t GetNumberOfLasers() const { return laser_emitters_.size(); }

  inline void SetLineOfSightDistance_m(const double line_of_sight_distance_m) { line_of_sight_distance_m_ = line_of_sight_distance_m; }
  inline double GetLineOfSightDistance_m() const { return line_of_sight_distance_m_; }

  inline void SetIsLaserReceivedByQpdSensor(const size_t id, const bool is_laser_received_by_qpd_sensor) {
    is_laser_received_by_qpd_sensors_[id] = is_laser_received_by_qpd_sensor;
  }
  inline std::vector<bool> GetIsLaserReceivedByQpdSensor() const { return is_laser_received_by_qpd_sensors_; }

  inline void SetDisplacementCalcedByQpdSensor_m(const size_t id, const double y_axis_displacement_m, const double z_axis_displacement_m) {
    qpd_positioning_sensor_y_axis_displacement_m_[id] = y_axis_displacement_m;
    qpd_positioning_sensor_z_axis_displacement_m_[id] = z_axis_displacement_m;
  }
  inline libra::Vector<2> GetYAxisDisplacementCalcedByQpdSensor_m() const { return qpd_positioning_sensor_y_axis_displacement_m_; }
  inline libra::Vector<2> GetZAxisDisplacementCalcedByQpdSensor_m() const { return qpd_positioning_sensor_z_axis_displacement_m_; }

 private:
  std::vector<CornerCubeReflector*> corner_cube_reflectors_;
  std::vector<LaserEmitter*> laser_emitters_;
  double line_of_sight_distance_m_ = 0.0;
  std::vector<bool> is_laser_received_by_qpd_sensors_ = {false, false};
  libra::Vector<2> qpd_positioning_sensor_y_axis_displacement_m_{0.0};
  libra::Vector<2> qpd_positioning_sensor_z_axis_displacement_m_{0.0};
};

#endif  // S2E_SIMULATION_FF_SPACECRAFT_INTER_SPACECRAFT_COMMUNICATION_HPP_
