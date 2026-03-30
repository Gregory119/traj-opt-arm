cmake_minimum_required(VERSION 3.16)
project(traj_opt_arm LANGUAGES CXX)

########### build fails without this #########
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
##############################################

# find libraries that should already be installed
find_package(Threads REQUIRED)

find_package(ifopt REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
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

# --- manually add rapidcsv library ---
include(FetchContent)

# Fetch csv-parser and make it available to my-executable
FetchContent_Declare(
  rapidcsv
  GIT_REPOSITORY https://github.com/d99kris/rapidcsv
  GIT_SHALLOW TRUE 
  GIT_TAG v8.92
) 

FetchContent_MakeAvailable(rapidcsv)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/trajectory_optimization)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ipopt)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ifopt)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/hardware)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/simulation)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cartpole)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/so101_arm)
