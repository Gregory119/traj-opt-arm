# create library
add_library(traj_utils STATIC trapezoidal_traj_extractor.cpp)
target_link_libraries(traj_utils PUBLIC Eigen3::Eigen pinocchio::pinocchio splines)

# Specify the include directories
target_include_directories(traj_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
