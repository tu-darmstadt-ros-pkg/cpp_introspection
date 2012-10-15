project(cpp_introspection)
cmake_minimum_required(VERSION 2.8)
find_package(catkin REQUIRED)
catkin_stack()

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(rosconsole REQUIRED)

set(${PROJECT_NAME}_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} include)
configure_file(cmake/introspection.cmake.in ${PROJECT_SOURCE_DIR}/cmake/introspection.cmake @ONLY)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES cpp_introspection
  CFG_EXTRAS introspection.cmake
)

#set the default path for built executables to the "bin" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS})

include(cmake/introspection.cmake)
add_subdirectory(src)

install(FILES stack.xml
        DESTINATION share/${PROJECT_NAME})
