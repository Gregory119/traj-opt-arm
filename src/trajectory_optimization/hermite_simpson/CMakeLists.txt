# Define the static library target
add_library(hermite_simpson STATIC hermite_simpson_collocation_constraints.cpp control_effort_hs_cost.cpp)
target_link_libraries(hermite_simpson PUBLIC traj_vars ifopt::ifopt_ipopt)
# Include header files that will be publically available to the target that
# links to this library.
target_include_directories(hermite_simpson PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
