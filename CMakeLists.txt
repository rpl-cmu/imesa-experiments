project(imesa_experiments CXX C)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)

message(STATUS "================  CMake iMESA Experiments  ======================")

# Dependency - GTSAM
find_package(GTSAM 4.2.0 REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# Dependency - The actual iMESA implementation and the IncrementalSAMAgent interface
include(FetchContent)
FetchContent_Declare(
  imesa
  GIT_REPOSITORY  https://github.com/rpl-cmu/imesa.git
  GIT_TAG         main
)
FetchContent_MakeAvailable(imesa)

# Build the methods we evaluate
add_subdirectory(prior_work)

# Build the experiment infrastructure
add_subdirectory(experiments)
