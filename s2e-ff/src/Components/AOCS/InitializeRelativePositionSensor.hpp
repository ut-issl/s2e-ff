#pragma once

#include "RelativePositionSensor.hpp"

RelativePositionSensor InitializeRelativePositionSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info, const Dynamics& dynamics);
