find_package(ROS REQUIRED COMPONENTS rosconsole)
include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(${Boost_INCLUDE_DIRS})

add_library(cpp_introspection SHARED introspection.cpp types.cpp)
target_link_libraries(cpp_introspection dl ${Boost_FILESYSTEM_LIBRARY} ${ROS_LIBRARIES})
install(TARGETS cpp_introspection DESTINATION lib)

option(BUILD_TEST "If true, some introspection libraries for std_msgs and a small test executable is built." OFF)
if(BUILD_TEST)
  introspection_add(std_msgs)
  add_executable(introspection_test test.cpp)
  target_link_libraries(introspection_test cpp_introspection)
endif(BUILD_TEST)
