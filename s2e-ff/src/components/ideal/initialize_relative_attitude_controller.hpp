#pragma once

#include "relative_attitude_controller.hpp"

RelativeAttitudeController InitializeRelativeAttitudeController(ClockGenerator* clock_gen, const std::string file_name,
                                                                const RelativeInformation& rel_info,
                                                                const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics,
                                                                const int reference_sat_id_input = 0);
