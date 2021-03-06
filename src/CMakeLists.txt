cmake_minimum_required(VERSION 3.0)
project(Diff)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin/")
add_executable(viewer viewer.cpp mesh.cpp)
add_executable(sim main.cpp mesh.cpp)

find_package(OpenMP REQUIRED)
target_link_libraries(viewer OpenMP::OpenMP_CXX)
target_link_libraries(sim OpenMP::OpenMP_CXX)

find_package(OpenGL REQUIRED)
target_include_directories(viewer PUBLIC ${OPENGL_INCLUDE_DIRS})
target_link_libraries(viewer ${OPENGL_LIBRARIES})
target_include_directories(sim PUBLIC ${OPENGL_INCLUDE_DIRS})
target_link_libraries(sim ${OPENGL_LIBRARIES})

find_package(GLEW REQUIRED)
target_include_directories(viewer PUBLIC ${GLEW_INCLUDE_DIRS})
target_link_libraries(viewer ${GLEW_LIBRARIES})
target_include_directories(sim PUBLIC ${GLEW_INCLUDE_DIRS})
target_link_libraries(sim ${GLEW_LIBRARIES})

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
target_include_directories(viewer PUBLIC ${GLFW_INCLUDE_DIRS})
target_link_libraries(viewer ${GLFW_LIBRARIES})
target_include_directories(sim PUBLIC ${GLFW_INCLUDE_DIRS})
target_link_libraries(sim ${GLFW_LIBRARIES})
