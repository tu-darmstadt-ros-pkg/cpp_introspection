add_library(cpp_introspection SHARED introspection.cpp types.cpp)
target_link_libraries(cpp_introspection dl ${Boost_FILESYSTEM_LIBRARY} ${rosconsole_LIBRARIES})

option(BUILD_TEST "If true, some introspection libraries for std_msgs and a small test executable is built." OFF)
if(BUILD_TEST)
  include(${PROJECT_SOURCE_DIR}/cmake/introspection.cmake)
  introspection_add(std_msgs)

  add_executable(introspection_test test.cpp)
  target_link_libraries(introspection_test cpp_introspection)
endif(BUILD_TEST)
