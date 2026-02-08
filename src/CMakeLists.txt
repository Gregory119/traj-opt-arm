# Optional: Set the path to the MuJoCo installation folder if it's not in a standard location.
# Replace "/opt/mujoco" with your actual MuJoCo installation path.
# set(MUJOCO_FOLDER "/opt/mujoco/lib/cmake")
#set(MUJOCO_FOLDER "/home/greg/dev/repos/mujoco/install/lib/cmake/mujoco")
#export CMAKE_PREFIX_PATH=/home/greg/dev/repos/mujoco/install/lib/cmake/mujoco

# Find the MuJoCo package. If MUJOCO_FOLDER is set, it will search there first.
# Otherwise, it relies on CMAKE_PREFIX_PATH or standard system paths.
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

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
add_executable(traj_opt_arm main.cpp simulator.cpp periodic_sim_timer.cpp)
target_include_directories(traj_opt_arm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ipopt)
