/**
 * @file initialize_relative_attitude_sensor.hpp
 * @brief Initialize function for RelativeAttitudeSensor
 */

#ifndef S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_ATTITUDE_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_ATTITUDE_SENSOR_HPP_

#include "relative_attitude_sensor.hpp"

/**
 * @fn InitializeRelativeAttitudeSensor
 * @brief Initialize function
 */
RelativeAttitudeSensor InitializeRelativeAttitudeSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics,
                                                        const int reference_sat_id_input = -1);

#endif  // S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_ATTITUDE_SENSOR_HPP_
