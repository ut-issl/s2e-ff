/**
 * @file initialize relative_attitude_controller.hpp
 * @brief Initialize functions for RelativeAttitudeController
 */

#ifndef S2E_COMPONENTS_IDEAL_INITIALIZE_RELATIVE_ATTITUDE_CONTROLLER_HPP_
#define S2E_COMPONENTS_IDEAL_INITIALIZE_RELATIVE_ATTITUDE_CONTROLLER_HPP_

#include "relative_attitude_controller.hpp"

/**
 * @fn InitializeRelativeAttitudeController
 * @brief Initialize function
 */
RelativeAttitudeController InitializeRelativeAttitudeController(ClockGenerator* clock_gen, const std::string file_name,
                                                                const RelativeInformation& rel_info,
                                                                const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics,
                                                                const int reference_sat_id_input = 0);

#endif  // S2E_COMPONENTS_IDEAL_INITIALIZE_RELATIVE_ATTITUDE_CONTROLLER_HPP_
