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
include_directories(${S2E_CORE_DIR}/src/Library/math)
include_directories(${S2E_CORE_DIR}/src/Library/utils)
include_directories(${S2E_CORE_DIR}/src/Dynamics)
include_directories(${S2E_CORE_DIR}/src/Disturbance)
include_directories(${S2E_CORE_DIR}/src/Interface/InitInput)
include_directories(${S2E_CORE_DIR}/src/Interface/LogOutput)
include_directories(${S2E_CORE_DIR}/src/Interface/SpacecraftInOut/Ports)
include_directories(${S2E_CORE_DIR}/src/Simulation)
include_directories(${S2E_CORE_DIR}/src/Simulation/MCSim)
include_directories(${S2E_CORE_DIR}/src/Simulation/Spacecraft)
include_directories(${S2E_CORE_DIR}/src/Simulation/Spacecraft/Structure)
include_directories(${S2E_CORE_DIR}/src/Simulation/GroundStation)
include_directories(${S2E_CORE_DIR}/src/Simulation/Case)
include_directories(${S2E_CORE_DIR}/src/Disturbance)
include_directories(${S2E_CORE_DIR}/src/Environment/Global)
include_directories(${S2E_CORE_DIR}/src/Environment/Local)
include_directories(${S2E_CORE_DIR}/src/Component)
include_directories(${S2E_CORE_DIR}/src/Component/Abstract)
include_directories(${S2E_CORE_DIR}/src/Component/AOCS)
include_directories(${S2E_CORE_DIR}/src/Component/CDH)
include_directories(${S2E_CORE_DIR}/src/Component/CommGS)
include_directories(${S2E_CORE_DIR}/src/Component/Mission/Telescope)
include_directories(${S2E_CORE_DIR}/src/Component/Power)
include_directories(${S2E_CORE_DIR}/src/Component/Propulsion)
include_directories(${S2E_CORE_DIR}/src/Component/Thermal)
include_directories(${S2E_CORE_DIR}/src/RelativeInformation)

## add_subdirectories
add_subdirectory(${S2E_CORE_DIR}/src/Component S2E_CORE/Component)
add_subdirectory(${S2E_CORE_DIR}/src/Disturbance S2E_CORE/Disturbance)
add_subdirectory(${S2E_CORE_DIR}/src/Dynamics S2E_CORE/Dynamics)
add_subdirectory(${S2E_CORE_DIR}/src/Environment/Global S2E_CORE/Environment/Global)
add_subdirectory(${S2E_CORE_DIR}/src/Environment/Local S2E_CORE/Environment/Local)
add_subdirectory(${S2E_CORE_DIR}/src/RelativeInformation S2E_CORE/RelativeInformation)
add_subdirectory(${S2E_CORE_DIR}/src/Interface/InitInput S2E_CORE/InitInput)
add_subdirectory(${S2E_CORE_DIR}/src/Interface/LogOutput S2E_CORE/LogOutput)
add_subdirectory(${S2E_CORE_DIR}/src/Interface/SpacecraftInOut S2E_CORE/SpacecraftInOut)
add_subdirectory(${S2E_CORE_DIR}/src/Interface/HilsInOut S2E_CORE/HilsInOut)
add_subdirectory(${S2E_CORE_DIR}/src/Library/igrf S2E_CORE/igrf)
add_subdirectory(${S2E_CORE_DIR}/src/Library/inih S2E_CORE/inih)
add_subdirectory(${S2E_CORE_DIR}/src/Library/math S2E_CORE/math)
add_subdirectory(${S2E_CORE_DIR}/src/Library/nrlmsise00 S2E_CORE/nrlmsise00)
add_subdirectory(${S2E_CORE_DIR}/src/Library/sgp4 S2E_CORE/sgp4)
add_subdirectory(${S2E_CORE_DIR}/src/Library/utils S2E_CORE/utils)
add_subdirectory(${S2E_CORE_DIR}/src/Library/optics S2E_CORE/optics)
add_subdirectory(${S2E_CORE_DIR}/src/Library/RelativeOrbit S2E_CORE/RelativeOrbit)
add_subdirectory(${S2E_CORE_DIR}/src/Library/Orbit S2E_CORE/Orbit)
add_subdirectory(${S2E_CORE_DIR}/src/Library/Geodesy S2E_CORE/Geodesy)
add_subdirectory(${S2E_CORE_DIR}/src/Simulation S2E_CORE/Simulation)

set(SOURCE_FILES
  src/S2eFf.cpp
  src/Components/AOCS/RelativeDistanceSensor.cpp
  src/Components/AOCS/InitializeRelativeDistanceSensor.cpp
  src/Components/AOCS/RelativePositionSensor.cpp
  src/Components/AOCS/InitializeRelativePositionSensor.cpp
  src/Components/IdealComponents/ForceGenerator.cpp
  src/Components/IdealComponents/InitializeForceGenerator.cpp
  src/Components/IdealComponents/RelativeAttitudeController.cpp
  src/Components/IdealComponents/InitializeRelativeAttitudeController.cpp
  src/Library/math/DualQuaternion.cpp
  src/Library/math/RotationFirstDualQuaternion.cpp
  src/Library/math/TranslationFirstDualQuaternion.cpp
  src/Library/Orbit/QuasiNonsingularOrbitalElements.cpp
  src/Library/RelativeOrbit/QuasiNonsingularRelativeOrbitalElements.cpp
  src/Library/RelativeOrbit/QuasiNonsingularOrbitalElementDifferences.cpp
  src/Simulation/Case/FfCase.cpp
  src/Simulation/Spacecraft/FfSat.cpp
  src/Simulation/Spacecraft/FfComponents.cpp
  src/Simulation/Spacecraft/FfComponents2.cpp
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
target_link_libraries(COMPONENT DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT SC_IO RELATIVE_INFO ${S2E_LIBRARIES})
target_link_libraries(DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT SIMULATION ${S2E_LIBRARIES})
target_link_libraries(DISTURBANCE DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT ${S2E_LIBRARIES})
target_link_libraries(SIMULATION DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT DISTURBANCE ${S2E_LIBRARIES})
target_link_libraries(GLOBAL_ENVIRONMENT ${CSPICE_LIB} ${S2E_LIBRARIES})
target_link_libraries(LOCAL_ENVIRONMENT GLOBAL_ENVIRONMENT ${CSPICE_LIB} ${S2E_LIBRARIES})
target_link_libraries(WRAPPER_NRLMSISE00 ${NRLMSISE00_LIB})

target_link_libraries(${PROJECT_NAME} DYNAMICS)
target_link_libraries(${PROJECT_NAME} DISTURBANCE)
target_link_libraries(${PROJECT_NAME} SIMULATION)
target_link_libraries(${PROJECT_NAME} GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT)
target_link_libraries(${PROJECT_NAME} RELATIVE_INFO)
target_link_libraries(${PROJECT_NAME} INI_ACC LOG_OUT SC_IO)
target_link_libraries(${PROJECT_NAME} COMPONENT)
target_link_libraries(${PROJECT_NAME} HILS_IO)

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
    src/Library/math/DualQuaternion.cpp
    test/TestDualQuaternion.cpp
    src/Library/math/RotationFirstDualQuaternion.cpp
    test/TestRotationFirstDualQuaternion.cpp
    src/Library/math/TranslationFirstDualQuaternion.cpp
    test/TestTranslationFirstDualQuaternion.cpp
  )
  add_executable(${TEST_PROJECT_NAME} ${TEST_FILES})
  target_link_libraries(${TEST_PROJECT_NAME} gtest gtest_main)
  target_link_libraries(${TEST_PROJECT_NAME} MATH)
  include_directories(${TEST_PROJECT_NAME})
  add_test(NAME s2e-test COMMAND ${TEST_PROJECT_NAME})
  enable_testing()
endif()
