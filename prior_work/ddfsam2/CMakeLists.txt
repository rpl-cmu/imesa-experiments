message(STATUS "================  DDFSAM2  ======================")

file(GLOB_RECURSE ddfsam2_srcs "*.cpp" "*.h")
add_library(ddfsam2 ${ddfsam2_srcs})
target_link_libraries(ddfsam2 imesa gtsam incremental_baseline)
target_include_directories(ddfsam2 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")