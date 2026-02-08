add_executable(main_ipopt_example cpp_example.cpp MyNLP.cpp)
include_directories(main_ipopt_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_ipopt_example ipopt)
