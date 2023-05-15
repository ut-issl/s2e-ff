// Simulator includes
#include <library/logger/logger.hpp>

// Add custom include files
#include "./simulation/Case/FfCase.hpp"

// degub print of initialize file path
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
  std::string ini_file = "../../data/ini/FfSimBase.ini";

  std::cout << "Starting simulation..." << std::endl;
  std::cout << "\tIni file: ";
  print_path(ini_file);

  auto simcase = FfCase(ini_file);
  simcase.Initialize();
  simcase.Main();

  return EXIT_SUCCESS;
}
