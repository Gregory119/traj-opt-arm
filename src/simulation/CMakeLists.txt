# find libraries that should already be installed
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

# sim library
add_library(sim STATIC simulator.cpp periodic_sim_timer.cpp)
target_include_directories(sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
target_link_libraries(sim mujoco::mujoco glfw traj_utils)

# sim executeable
add_executable(sim_arm main.cpp)
target_link_libraries(
  sim_arm
  sim
)
