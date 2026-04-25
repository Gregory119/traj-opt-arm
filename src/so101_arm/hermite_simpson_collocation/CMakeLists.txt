add_executable(main_so101_hs main_so101_hs.cpp)
include_directories(main_so101_hs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_so101_hs PRIVATE Boost::timer ipopt hermite_simpson traj_utils robot_dynamics
sim so101_bus)
