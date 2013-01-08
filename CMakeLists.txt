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

find_package(ROS REQUIRED COMPONENTS rosconsole)
include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include)
add_subdirectory(src)

install(FILES stack.xml DESTINATION ${PROJECT_SHARE_INSTALL_PREFIX})
install(FILES manifest.xml DESTINATION ${PROJECT_SHARE_INSTALL_PREFIX})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY src/ DESTINATION ${PROJECT_SHARE_INSTALL_PREFIX}/src)
