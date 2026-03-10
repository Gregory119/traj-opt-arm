# Define the static library target
add_library(splines STATIC quadratic_spline.cpp)
target_link_libraries(splines PUBLIC Eigen3::Eigen)
# Include header files that will be publically available to the target that
# links to this library.
target_include_directories(splines PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
