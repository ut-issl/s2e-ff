#pragma once

#include "RelativeAttitudeController.hpp"

RelativeAttitudeController InitializeRelativeAttitudeController(ClockGenerator* clock_gen, const std::string file_name,
                                                                const RelativeInformation& rel_info,
                                                                const LocalCelestialInformation& local_celes_info, const Dynamics& dynamics);
