## General Settings
cmake_policy(SET CMP0048 NEW)
project(S2E_FF
  LANGUAGES CXX
  DESCRIPTION "s2e-ff: A sample of formation flying simulation with S2E"
  VERSION 0.0.0
)

cmake_minimum_required(VERSION 3.13)

# build config
option(BUILD_64BIT "Build 64bit" OFF)
option(GOOGLE_TEST "Execute GoogleTest" OFF)
if (NOT BUILD_64BIT)
  option(GOOGLE_TEST OFF) # GoogleTest supports 64bit only
endif()

# preprocessor
if(WIN32)
  add_definitions(-DWIN32)
endif()

## set directory path
set(S2E_CORE_DIR ../s2e-core)
if(NOT DEFINED EXT_LIB_DIR)
  set(EXT_LIB_DIR ../ExtLibraries)
endif()
set(CSPICE_DIR ${EXT_LIB_DIR}/cspice)
set(NRLMSISE00_DIR ${EXT_LIB_DIR}/nrlmsise00)

## include directories of s2e-core
include_directories(${CSPICE_DIR}/include)
include_directories(${NRLMSISE00_DIR}/src)
include_directories(${S2E_CORE_DIR}/src)

## add_subdirectories
add_subdirectory(${S2E_CORE_DIR}/src/components s2e_core/components)
add_subdirectory(${S2E_CORE_DIR}/src/disturbances s2e_core/disturbances)
add_subdirectory(${S2E_CORE_DIR}/src/dynamics s2e_core/dynamics)
add_subdirectory(${S2E_CORE_DIR}/src/environment/global s2e_core/environment/global)
add_subdirectory(${S2E_CORE_DIR}/src/environment/local s2e_core/environment/local)
add_subdirectory(${S2E_CORE_DIR}/src/library s2e_core/library)
add_subdirectory(${S2E_CORE_DIR}/src/simulation s2e_core/simulation)


set(SOURCE_FILES
  src/s2e_ff.cpp
  src/components/aocs/relative_distance_sensor.cpp
  src/components/aocs/initialize_relative_distance_sensor.cpp
  src/components/aocs/relative_position_sensor.cpp
  src/components/aocs/initialize_relative_position_sensor.cpp
  src/components/aocs/relative_velocity_sensor.cpp
  src/components/aocs/initialize_relative_velocity_sensor.cpp
  src/components/ideal/relative_attitude_controller.cpp
  src/components/ideal/initialize_relative_attitude_controller.cpp
  src/components/controller/relative_orbit_controller_deputy.cpp
  src/components/controller/relative_orbit_controller_chief.cpp
  src/components/analyzer/relative_orbit_analyzer.cpp
  src/library/math/dual_quaternion.cpp
  src/library/math/rotation_first_dual_quaternion.cpp
  src/library/math/translation_first_dual_quaternion.cpp
  src/library/orbit/quasi_nonsingular_orbital_elements.cpp
  src/library/relative_orbit/quasi_nonsingular_relative_orbital_elements.cpp
  src/library/relative_orbit/quasi_nonsingular_orbital_element_differences.cpp
  src/simulation/case/ff_case.cpp
  src/simulation/spacecraft/ff_satellite.cpp
  src/simulation/spacecraft/ff_components.cpp
  src/simulation/spacecraft/ff_components_2.cpp
)
# Create executable file
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

## cspice library
if(CYGWIN)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")
  set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_cygwin/lib)
elseif(UNIX)
  if(BUILD_64BIT)
    set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_unix64/lib)
  else()
    set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_unix/lib)
  endif()
endif()

if(WIN32)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  find_library(CSPICE_LIB
    NAMES cspice.lib csupport.lib
    PATHS ${CSPICE_DIR}/cspice_msvs/lib)
else()
  find_library(CSPICE_LIB
    NAMES cspice.a csupport.a
    PATHS ${CSPICE_LIB_DIR})
endif()
if(NOT CSPICE_LIB)
  message(FATAL_ERROR "cspice not found in ${EXT_LIB_DIR}")
endif()

## nrlmsise00 library
if(CYGWIN)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  find_library(NRLMSISE00_LIB 
    NAMES libnrlmsise00.a
    PATHS ${NRLMSISE00_DIR}/lib)
elseif(UNIX)
  if(BUILD_64BIT)
    find_library(NRLMSISE00_LIB
      NAMES libnrlmsise00.a
      PATHS ${NRLMSISE00_DIR}/lib64)
  else()
    find_library(NRLMSISE00_LIB
      NAMES libnrlmsise00.a
      PATHS ${NRLMSISE00_DIR}/lib)
  endif()
elseif(WIN32)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  find_library(NRLMSISE00_LIB 
    NAMES libnrlmsise00.lib
    PATHS ${NRLMSISE00_DIR}/lib)
endif()
if(NOT NRLMSISE00_LIB)
  message(FATAL_ERROR "nrlmsise00 not found in ${EXT_LIB_DIR}")
endif()

## Linking libraries
set(S2E_LIBRARIES
  IGRF WRAPPER_NRLMSISE00 INIH SGP4 UTIL OPTICS RELATIVE_ORBIT_MODELS ORBIT_MODELS GEODESY MATH
)
# Initialize link
target_link_libraries(COMPONENT DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT LIBRARY)
target_link_libraries(DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT SIMULATION LIBRARY)
target_link_libraries(DISTURBANCE DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT LIBRARY)
target_link_libraries(SIMULATION DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT DISTURBANCE LIBRARY)
target_link_libraries(GLOBAL_ENVIRONMENT ${CSPICE_LIB} LIBRARY)
target_link_libraries(LOCAL_ENVIRONMENT GLOBAL_ENVIRONMENT ${CSPICE_LIB} LIBRARY)
target_link_libraries(LIBRARY ${NRLMSISE00_LIB})

target_link_libraries(${PROJECT_NAME} DYNAMICS)
target_link_libraries(${PROJECT_NAME} DISTURBANCE)
target_link_libraries(${PROJECT_NAME} SIMULATION)
target_link_libraries(${PROJECT_NAME} GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT)
target_link_libraries(${PROJECT_NAME} COMPONENT)


## Cmake debug
message("Cspice_LIB:  " ${CSPICE_LIB})
message("nrlmsise00_LIB:  " ${NRLMSISE00_LIB})

include(${S2E_CORE_DIR}/common.cmake)

## GoogleTest settings
if(GOOGLE_TEST)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
  )
  FetchContent_MakeAvailable(googletest)
  if(WIN32)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  endif()

  ## Unit test
  set(TEST_PROJECT_NAME ${PROJECT_NAME}_TEST)
  set(TEST_FILES
    # Dual Quaternion
    src/library/math/dual_quaternion.cpp
    test/test_dual_quaternion.cpp
    src/library/math/rotation_first_dual_quaternion.cpp
    test/test_rotation_first_dual_quaternion.cpp
    src/library/math/translation_first_dual_quaternion.cpp
    test/test_translation_first_dual_quaternion.cpp
    # Orbital elements
    src/library/orbit/quasi_nonsingular_orbital_elements.cpp
    test/test_quasi_nonsingular_orbital_elements.cpp
    # Relative orbital elements
    src/library/relative_orbit/quasi_nonsingular_orbital_element_differences.cpp
    test/test_quasi_nonsingular_orbital_element_differences.cpp
    src/library/relative_orbit/quasi_nonsingular_relative_orbital_elements.cpp
    test/test_quasi_nonsingular_relative_orbital_elements.cpp
  )
  add_executable(${TEST_PROJECT_NAME} ${TEST_FILES})
  target_link_libraries(${TEST_PROJECT_NAME} gtest gtest_main)
  target_link_libraries(${TEST_PROJECT_NAME} LIBRARY)
  include_directories(${TEST_PROJECT_NAME})
  add_test(NAME s2e-test COMMAND ${TEST_PROJECT_NAME})
  enable_testing()
endif()
