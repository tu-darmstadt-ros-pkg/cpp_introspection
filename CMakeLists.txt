project(cpp_introspection)
cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED COMPONENTS rosconsole)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cpp_introspection
  CATKIN_DEPENDS rosconsole rostime roscpp_traits roscpp_serialization
  CFG_EXTRAS introspection.cmake
)
include(${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/introspection.cmake)

find_package(Boost REQUIRED COMPONENTS filesystem)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_subdirectory(src)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY src/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)
