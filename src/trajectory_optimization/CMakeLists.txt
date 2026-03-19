# Define the static library target
add_library(traj_vars INTERFACE)
target_link_libraries(traj_vars INTERFACE ifopt::ifopt_ipopt)
# Include header files that will be publically available to the target that
# links to this library.
target_include_directories(traj_vars INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/splines)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dynamics)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/trapezoidal)
