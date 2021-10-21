cmake_minimum_required(VERSION 3.10.2)

project(matrix_test)

#Target; list source files and source with main file
add_executable(main src/main_file.cpp math/kalman_filter.cpp math/matrix.cpp)

set_target_properties(main PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/math ${CMAKE_CURRENT_SOURCE_DIR}/model)
