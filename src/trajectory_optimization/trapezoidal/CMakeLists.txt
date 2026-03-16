# Define the static library target
add_library(trapezoidal STATIC trapezoidal_collocation_constraints.cpp control_effort_trapezoidal_cost.cpp)
target_link_libraries(trapezoidal PUBLIC traj_vars ifopt::ifopt_ipopt)
# Include header files that will be publically available to the target that
# links to this library.
target_include_directories(trapezoidal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
