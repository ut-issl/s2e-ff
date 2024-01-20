/**
 * @file ff_components_2.hpp
 * @brief Components management installed on a FF spacecraft
 */

#ifndef S2E_FF_SIMULATION_SPACECRAFT_FF_COMPONENTS_2_HPP_
#define S2E_FF_SIMULATION_SPACECRAFT_FF_COMPONENTS_2_HPP_

#include <environment/global/global_environment.hpp>
#include <environment/local/local_environment.hpp>
#include <simulation/spacecraft/installed_components.hpp>

#include "../case/ff_inter_spacecraft_communication.hpp"

// include for components
#include <components/ideal/force_generator.hpp>
#include <components/real/cdh/on_board_computer.hpp>

#include "../../components/aocs/corner_cube_reflector.hpp"
#include "../../components/aocs/information_parser.hpp"
#include "../../components/aocs/initialize_relative_distance_sensor.hpp"
#include "../../components/aocs/initialize_relative_position_sensor.hpp"
#include "../../components/aocs/laser_emitter.hpp"
#include "../../components/aocs/qpd_positioning_sensor.hpp"
#include "../../components/aocs/relative_attitude_sensor.hpp"
#include "../../components/ideal/initialize_relative_attitude_controller.hpp"

/**
 * @class FfComponents2
 * @brief A components class for FF satellite
 */
class FfComponents2 : public InstalledComponents {
 public:
  /**
   * @fn FfComponents
   * @brief Constructor
   */
  FfComponents2(const Dynamics* dynamics, const Structure* structure, const LocalEnvironment* local_env, const GlobalEnvironment* glo_env,
                const SimulationConfiguration* config, ClockGenerator* clock_gen, const RelativeInformation* rel_info,
                FfInterSpacecraftCommunication& inter_spacecraft_communication);
  /**
   * @fn ~FfComponents
   * @brief Destructor
   */
  ~FfComponents2();

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
  OnBoardComputer* obc_;  //!< On board computer
  std::vector<CornerCubeReflector*> corner_cube_reflectors_;
  std::vector<QpdPositioningSensor*> qpd_positioning_sensors_;

  // References
  const Dynamics* dynamics_;               //!< Dynamics information of the spacecraft
  const Structure* structure_;             //!< Structure information of the spacecraft
  const LocalEnvironment* local_env_;      //!< Local environment information around the spacecraft
  const GlobalEnvironment* glo_env_;       //!< Global environment information
  const SimulationConfiguration* config_;  //!< Simulation settings
  const RelativeInformation* rel_info_;    //!< Relative information
  InformationParser* information_parser_;
  FfInterSpacecraftCommunication& inter_spacecraft_communication_;
};

#endif  // S2E_FF_SIMULATION_SPACECRAFT_FF_COMPONENTS_2_HPP_
