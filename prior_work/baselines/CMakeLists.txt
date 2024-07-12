message(STATUS "================  Incremental Baselines  ======================")

file(GLOB_RECURSE incremental_baseline_srcs "*.cpp" "*.h")
add_library(incremental_baseline ${incremental_baseline_srcs})
target_link_libraries(incremental_baseline imesa gtsam)
target_include_directories(incremental_baseline PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")