message(STATUS "================  Experiment Runner  ======================")

# Prereqs
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(jrl REQUIRED)
find_package(nlohmann_json REQUIRED)

# Build the experiments module
file(GLOB_RECURSE experiments_srcs "*.cpp" "*.h")
add_library(experiments ${experiments_srcs})
target_link_libraries(experiments jrl gtsam imesa ddfsam2)
target_include_directories(experiments PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Main Entry Point
add_executable(run-trial run-trial.cpp)
target_link_libraries(run-trial PUBLIC ${Boost_LIBRARIES} jrl experiments)