###### std::numbers kept failing until I added this

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 
set(CMAKE_CXX_EXTENSIONS OFF)
#####################################################

find_package(ifopt REQUIRED)
find_package(pinocchio REQUIRED)


add_executable(main_cartpole_HS main_cartpole_HS.cpp HermiteSimpson_collocation_constraints.cpp control_effort_HS_cost.cpp)
include_directories(main_cartpole_HS PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_cartpole_HS PRIVATE ipopt ifopt::ifopt_ipopt pinocchio::pinocchio)

add_executable(main_load_cartpole main_load_cartpole_urdf.cpp)
include_directories(main_load_cartpole PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_load_cartpole PRIVATE pinocchio::pinocchio)
