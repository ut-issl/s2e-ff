#include "InitializeRelativeDistanceSensor.hpp"

#include <Interface/InitInput/IniAccess.h>

#include "../Abstract/InitializeSensorBase.hpp"

RelativeDistanceSensor InitializeRelativeDistanceSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s,
                                                        const RelativeInformation& rel_info) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("ComponentBase", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // SensorBase
  SensorBase<1> sensor_base = ReadSensorBaseInformation<1>(file_name, compo_step_time_s * (double)(prescaler));

  // RelativeDistanceSensor
  char section[30] = "RelativeDistanceSensor";
  int target_sat_id = ini_file.ReadInt(section, "target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");

  RelativeDistanceSensor relative_distance_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, rel_info);

  return relative_distance_sensor;
}
