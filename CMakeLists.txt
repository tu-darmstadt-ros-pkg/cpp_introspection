project(cpp_introspection)
cmake_minimum_required(VERSION 2.8)
find_package(catkin REQUIRED)
catkin_stack()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND DEFINED ENV{ROS_WORKSPACE})
  set(CMAKE_INSTALL_PREFIX "$ENV{ROS_WORKSPACE}/install" CACHE PATH "Install prefix" FORCE)
endif()

find_package(Boost REQUIRED COMPONENTS filesystem)
set(${PROJECT_NAME}_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} include)
include_directories(${ROS_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS})

#configure_file(cmake/introspection.cmake.in ${PROJECT_SOURCE_DIR}/cmake/introspection.cmake @ONLY)

catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES cpp_introspection
  CFG_EXTRAS introspection.cmake
)

#set the default path for built executables to the "bin" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME})

add_subdirectory(src)

install(FILES stack.xml DESTINATION share/${PROJECT_NAME})
install(FILES manifest.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
