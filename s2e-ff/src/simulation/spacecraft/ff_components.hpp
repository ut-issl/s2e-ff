/**
 * @file ff_components.hpp
 * @brief Components management installed on a FF spacecraft
 */

#ifndef S2E_FF_SIMULATION_SPACECRAFT_FF_COMPONENTS_HPP_
#define S2E_FF_SIMULATION_SPACECRAFT_FF_COMPONENTS_HPP_

#include <environment/global/global_environment.hpp>
#include <environment/local/local_environment.hpp>
#include <simulation/spacecraft/installed_components.hpp>

#include "../case/ff_inter_spacecraft_communication.hpp"

// include for components
#include <components/ideal/force_generator.hpp>
#include <components/real/cdh/on_board_computer.hpp>

#include "../../components/aocs/initialize_relative_attitude_sensor.hpp"
#include "../../components/aocs/initialize_relative_distance_sensor.hpp"
#include "../../components/aocs/initialize_relative_position_sensor.hpp"
#include "../../components/aocs/initialize_relative_velocity_sensor.hpp"
#include "../../components/aocs/laser_distance_meter.hpp"
#include "../../components/ideal/initialize_relative_attitude_controller.hpp"

/**
 * @class FfComponents
 * @brief A components class for FF satellite
 */
class FfComponents : public InstalledComponents {
 public:
  /**
   * @fn FfComponents
   * @brief Constructor
   */
  FfComponents(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
               const SimulationConfiguration* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info,
               FfInterSpacecraftCommunication& inter_spacecraft_communication);
  /**
   * @fn ~FfComponents
   * @brief Destructor
   */
  ~FfComponents();

  // Override functions for InstalledComponents
  /**
   * @fn GenerateForce_b_N
   * @brief Return force generated by components in unit Newton in body fixed frame
   */
  libra::Vector<3> GenerateForce_b_N();
  /**
   * @fn GenerateTorque_b_Nm
   * @brief Return torque generated by components in unit Newton-meter in body fixed frame
   */
  libra::Vector<3> GenerateTorque_b_Nm();
  /**
   * @fn LogSetup
   * @brief Setup the logger for components
   */
  void LogSetup(Logger& logger);

 private:
  // Components
  // CDH
  OnBoardComputer* obc_;  //!< On board computer
  // Sensors
  RelativeDistanceSensor* relative_distance_sensor_;  //!< Example of Relative distance sensor
  RelativePositionSensor* relative_position_sensor_;  //!< Example of Relative position sensor
  RelativeAttitudeSensor* relative_attitude_sensor_;  //!< Example of Relative attitude sensor
  RelativeVelocitySensor* relative_velocity_sensor_;  //!< Example of Relative velocity sensor
  LaserDistanceMeter* laser_distance_meter_;
  // Actuators
  ForceGenerator* force_generator_;                           //!< Example of force generator
  RelativeAttitudeController* relative_attitude_controller_;  //!< Example of attitude controller

  // References
  const Dynamics* dynamics_;               //!< Dynamics information of the spacecraft
  const Structure* structure_;             //!< Structure information of the spacecraft
  const LocalEnvironment* local_env_;      //!< Local environment information around the spacecraft
  const GlobalEnvironment* glo_env_;       //!< Global environment information
  const SimulationConfiguration* config_;  //!< Simulation settings
  const RelativeInformation* rel_info_;    //!< Relative information
  FfInterSpacecraftCommunication& inter_spacecraft_communication_;
};

#endif  // S2E_FF_SIMULATION_SPACECRAFT_FF_COMPONENTS_HPP_
