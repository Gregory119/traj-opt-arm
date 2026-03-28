find_package(Threads REQUIRED)


#paths
add_library(so101_bus STATIC
  so101_bus.cpp
  calibration.cpp
)

target_include_directories(so101_bus
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(so101_bus
  PUBLIC
    Threads::Threads
    traj_utils
)



#executables

add_executable(so101_trajectory_execution
  so101_trajectory_execution.cpp
)
target_link_libraries(so101_trajectory_execution PRIVATE so101_bus)



add_executable(read_servo_params
  read_servo_params.cpp
)
target_link_libraries(read_servo_params PRIVATE so101_bus)
