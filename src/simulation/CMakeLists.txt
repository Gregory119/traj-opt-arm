# find libraries that should already be installed
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(sim_arm main.cpp simulator.cpp periodic_sim_timer.cpp )
target_include_directories(sim_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
target_link_libraries(
  sim_arm
  mujoco::mujoco
  glfw
  ipopt
)
