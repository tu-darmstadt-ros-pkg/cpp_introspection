project(cpp_introspection)
cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED)
catkin_stack()

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include
  LIBRARIES cpp_introspection
  CFG_EXTRAS introspection.cmake
)
include(${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}/introspection.cmake)

include_directories(include)
add_subdirectory(src)

install(FILES stack.xml DESTINATION share/${PROJECT_NAME})
install(FILES manifest.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

