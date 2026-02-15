add_executable(main_ifopt_hs071 main_ifopt_hs071.cpp)
include_directories(main_ifopt_hs071 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_ifopt_hs071 PRIVATE ipopt ifopt::ifopt_ipopt)
