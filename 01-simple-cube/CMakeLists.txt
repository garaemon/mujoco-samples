cmake_minimum_required(VERSION 2.8.3)
project(mujoco_sample_simple_cube)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
add_executable(main main.cpp)
target_link_libraries(main mujoco::mujoco glfw OpenGL::GL)
target_include_directories(main PRIVATE /usr/include)
# Copy model.xml to build directory so that we can run main under build directory.
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/model.xml
  ${CMAKE_CURRENT_BINARY_DIR}/model.xml
  COPYONLY)
