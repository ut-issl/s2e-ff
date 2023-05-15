/**
 * @file s2e_ff.cpp
 * @brief The main file of S2E-FF
 */

// Simulator includes
#include <library/logger/logger.hpp>

// Add custom include files
#include "./simulation/case/ff_case.hpp"

// debug print of initialize file path
void print_path(std::string path) {
#ifdef WIN32
  std::cout << path << std::endl;
#else
  const char *rpath = realpath(path.c_str(), NULL);
  if (rpath) {
    std::cout << rpath << std::endl;
    free((void *)rpath);
  }
#endif
}

// Main function
int main() {
  // Set initialize file
  std::string ini_file = "../../data/initialize_files/ff_simulation_base.ini";

  std::cout << "Starting simulation..." << std::endl;
  std::cout << "\tIni file: ";
  print_path(ini_file);

  auto simulation_case = FfCase(ini_file);
  simulation_case.Initialize();
  simulation_case.Main();

  return EXIT_SUCCESS;
}
