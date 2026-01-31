cmake_minimum_required(VERSION 3.16)
project(TrajOptArm CXX)

set(CMAKE_CXX_STANDARD 17)

# Optional: Set the path to the MuJoCo installation folder if it's not in a standard location.
# Replace "/opt/mujoco" with your actual MuJoCo installation path.
# set(MUJOCO_FOLDER "/opt/mujoco/lib/cmake")
#set(MUJOCO_FOLDER "/home/greg/dev/repos/mujoco/install/lib/cmake/mujoco")
#export CMAKE_PREFIX_PATH=/home/greg/dev/repos/mujoco/install/lib/cmake/mujoco

# Find the MuJoCo package. If MUJOCO_FOLDER is set, it will search there first.
# Otherwise, it relies on CMAKE_PREFIX_PATH or standard system paths.
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

# Add your executable target
add_executable(traj_opt_arm src/main.cpp)

# Link your executable against the imported mujoco target
target_link_libraries(
  traj_opt_arm
  mujoco::mujoco
  glfw
)

# If your application uses windowing (e.g., for a viewer using GLFW), 
# you may need to find and link additional libraries like OpenGL and GLFW.
# find_package(OpenGL REQUIRED)
# target_link_libraries(my_app PUBLIC OpenGL::OpenGL glfw)
