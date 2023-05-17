/**
 * @file initialize_relative_velocity_sensor.hpp
 * @brief Initialize function for RelativeVelocitySensor
 */

#ifndef S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_POSITION_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_POSITION_SENSOR_HPP_

#include "relative_position_sensor.hpp"

/**
 * @fn InitializeRelativePositionSensor
 * @brief Initialize function
 */
RelativePositionSensor InitializeRelativePositionSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics,
                                                        const int reference_sat_id_input = -1);

#endif  // S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_POSITION_SENSOR_HPP_
