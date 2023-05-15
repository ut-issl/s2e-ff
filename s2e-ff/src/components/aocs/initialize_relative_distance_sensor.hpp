/**
 * @file initialize_relative_distance_sensor.hpp
 * @brief Relative distance sensor
 */

#ifndef S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_DISTANCE_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_DISTANCE_SENSOR_HPP_

#include "relative_distance_sensor.hpp"

/**
 * @fn InitializeRelativeDistanceSensor
 * @brief Initialize function
 */
RelativeDistanceSensor InitializeRelativeDistanceSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const int reference_sat_id_input = -1);

#endif  // S2E_COMPONENTS_AOCS_INITIALIZE_RELATIVE_DISTANCE_SENSOR_HPP_
