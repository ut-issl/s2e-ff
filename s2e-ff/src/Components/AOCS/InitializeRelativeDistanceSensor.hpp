#pragma once

#include "./RelativeDistanceSensor.hpp"

RelativeDistanceSensor InitializeRelativeDistanceSensor(ClockGenerator* clock_gen, const std::string file_name,
                                                        const double compo_step_time_s, const RelativeInformation& rel_info);

// TODO:これをコアに移して全体で共有したい
template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s);
