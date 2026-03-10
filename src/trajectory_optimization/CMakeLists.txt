add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR})

# header only library
add_library(traj_utils INTERFACE)

# Specify the include directories
target_include_directories(traj_utils INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
