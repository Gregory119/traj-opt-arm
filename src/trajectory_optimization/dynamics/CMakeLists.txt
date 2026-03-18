# create library
add_library(robot_dynamics STATIC robot_dynamics.cpp)
target_link_libraries(robot_dynamics PUBLIC Eigen3::Eigen pinocchio::pinocchio)

# Specify the include directories
target_include_directories(robot_dynamics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
