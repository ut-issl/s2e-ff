#include "InitializeRelativeDistanceSensor.hpp"

#include <Interface/InitInput/IniAccess.h>

RelativeDistanceSensor InitializeRelativeDistanceSensor(ClockGenerator* clock_gen, const std::string file_name,
                                                        const double compo_step_time_s, const RelativeInformation& rel_info) {
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

template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width) {
  IniAccess ini_file(file_name);
  char section[30] = "SensorBase";
  
  libra::Vector<N * N> scale_factor_vector;
  ini_file.ReadVector(section, "scale_factor_c", scale_factor_vector);
  libra::Matrix<N, N> scale_factor_c;
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      scale_factor_c[i][j] = scale_factor_vector[i * N + j];
    }
  }

  libra::Vector<N> constant_bias_c;
  ini_file.ReadVector(section, "constant_bias_c", constant_bias_c);
  Vector<N> nromal_random_standard_deviation_c;
  ini_file.ReadVector(section, "nromal_random_standard_deviation_c", nromal_random_standard_deviation_c);
  Vector<N> random_walk_standard_deviation_c;
  ini_file.ReadVector(section, "random_walk_standard_deviation_c", random_walk_standard_deviation_c);
  Vector<N> random_walk_limit_c;
  ini_file.ReadVector(section, "random_walk_limit_c", random_walk_limit_c);
  
  double range_to_const = ini_file.ReadDouble(section, "range_to_const");
  Vector<N> range_to_const_c{range_to_const};
  double range_to_zero = ini_file.ReadDouble(section, "range_to_zero");
  Vector<N> range_to_zero_c{range_to_zero};

  SensorBase<N> sensor_base(scale_factor_c, range_to_const_c, range_to_zero_c,
                            constant_bias_c, nromal_random_standard_deviation_c,
                            step_width, random_walk_standard_deviation_c, random_walk_limit_c);

  return sensor_base;
}
