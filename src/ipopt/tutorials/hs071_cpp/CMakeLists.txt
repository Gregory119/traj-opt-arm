add_executable(main_hs071 main_hs071.cpp hs071_nlp.cpp)
include_directories(main_hs071 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_hs071 ipopt)
