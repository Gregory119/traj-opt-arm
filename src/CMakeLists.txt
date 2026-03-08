cmake_minimum_required(VERSION 3.22)
project(RobotArmTrajectoryOptimization VERSION 1.0 LANGUAGES CXX)

# c++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find libraries that should already be installed
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

find_package(ifopt)
find_package(pinocchio REQUIRED)

# --- manually add IPOPT as a library ---
add_library(ipopt SHARED IMPORTED)

# Specify the location of the library file
set_property(
  TARGET ipopt
  PROPERTY IMPORTED_LOCATION 
  "/usr/lib/libipopt.so" 
)

# Specify the include directories for the headers
target_include_directories(
  ipopt
  INTERFACE 
  "/usr/include/coin/"
)

# globally define use of the C std library for IPOPT
add_compile_definitions(HAVE_CSTDDEF)
# --- IPOPT library added ---


#######################################################################
# Executeables
#######################################################################
add_executable(traj_opt_arm main.cpp simulator.cpp periodic_sim_timer.cpp )
target_include_directories(traj_opt_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
target_link_libraries(
  traj_opt_arm
  mujoco::mujoco
  glfw
  ipopt
)




# If your application uses windowing (e.g., for a viewer using GLFW), 
# you may need to find and link additional libraries like OpenGL and GLFW.
# find_package(OpenGL REQUIRED)
# target_link_libraries(my_app PUBLIC OpenGL::OpenGL glfw)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/trajectory_optimization)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ipopt)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ifopt)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cartpole)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/hardware)
