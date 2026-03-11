add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/splines)

# header only library
add_library(traj_utils INTERFACE)
target_link_libraries(traj_utils INTERFACE Eigen3::Eigen)

# Specify the include directories
target_include_directories(traj_utils INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
