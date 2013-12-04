if(NOT DEFINED package)
  message(SEND_FATAL "You have to define variable package to determine the package for which introspection library is going to be built.")
  return()
endif(NOT DEFINED package)
set(CMAKE_BUILD_TYPE MinSizeRel)

# Return if introspection target for package already exists
if(TARGET introspection_${package})
  return()
endif()

# List all messages in package ${package}
introspection_get_messages(${package} ${package}_MSGS)

# Return if message list is empty
if(NOT ${package}_MSGS)
  return()
endif()

# Reset variables
unset(introspection_${package}_INCLUDE_DIRS)
unset(introspection_${package}_LIBRARIES)
unset(${package}_MSG_DEPENDENCIES)

# Find message package
find_package(${package} QUIET)
if(${package}_FOUND)
  message(STATUS "Found ${package} at ${${package}_DIR}")
  list(APPEND introspection_${package}_INCLUDE_DIRS ${${package}_INCLUDE_DIRS})

else() # Fall back to rospack find...
  execute_process(
    COMMAND rospack find ${package}
    OUTPUT_VARIABLE ${package}_PACKAGE_PATH
    RESULT_VARIABLE ${package}_PACKAGE_PATH_RESULT
  )
  if(NOT ${${package}_PACKAGE_PATH_RESULT} EQUAL 0)
    message(SEND_ERROR "Could not find package ${package}")
    return()
  endif()
  string(REPLACE "\n" "" ${package}_PACKAGE_PATH ${${package}_PACKAGE_PATH})
  message(STATUS "Found ${package} at ${${package}_PACKAGE_PATH}")

  execute_process(
    COMMAND rospack cflags-only-I ${package}
    OUTPUT_VARIABLE ${package}_INCLUDE_DIRS
    RESULT_VARIABLE ${package}_INCLUDE_DIRS_RESULT
  )
  if(NOT ${${package}_INCLUDE_DIRS_RESULT} EQUAL 0)
    message(SEND_ERROR "Could not get include directories for package ${package}")
    return()
  endif()
  string(REPLACE "\n" ";" ${package}_INCLUDE_DIRS ${${package}_INCLUDE_DIRS})
  string(REPLACE " " ";" ${package}_INCLUDE_DIRS ${${package}_INCLUDE_DIRS})
  list(REMOVE_ITEM ${package}_INCLUDE_DIRS "")
  list(APPEND introspection_${package}_INCLUDE_DIRS ${${package}_INCLUDE_DIRS})
endif()

# Find and iterate through dependent packages
if(NOT DEFINED ${package}_MSG_DEPENDENCIES)
  execute_process(
    COMMAND rospack depends ${package}
    OUTPUT_VARIABLE ${package}_MSG_DEPENDENCIES
    RESULT_VARIABLE _result
  )
  if(NOT ${_result} EQUAL 0)
    message(SEND_ERROR "Could not get package dependencies for package ${package}.")
    return()
  endif()
  string(REPLACE "\n" ";" ${package}_MSG_DEPENDENCIES "${${package}_MSG_DEPENDENCIES}")
  list(REMOVE_ITEM ${package}_MSG_DEPENDENCIES "")
  #list(APPEND ${package}_MSG_DEPENDENCIES std_msgs)
  list(REMOVE_ITEM ${package}_MSG_DEPENDENCIES ${package})
endif()
#message(STATUS "Package ${package} has dependencies ${${package}_MSG_DEPENDENCIES}")

unset(_dependent_declarations)
unset(_dependent_initializers)
foreach(dependent_package ${${package}_MSG_DEPENDENCIES})
  #message(STATUS "Package ${dependent_package} is required for ${package}")

  # check if this package has messages
  introspection_get_messages(${dependent_package} ${dependent_package}_MSGS)

  if(${dependent_package}_MSGS)
    if(INTROSPECTION_ADD_RECURSIVE)
      introspection_add(${dependent_package} RECURSIVE)
    endif(INTROSPECTION_ADD_RECURSIVE)

    #set(_dependent_declarations "${_dependent_declarations}#include \"introspection/${dependent_package}/package.h\"\n")
    #set(_dependent_initializers "${_dependent_initializers}    ${dependent_package}::Instance();\n")
    #list(APPEND introspection_${package}_LIBRARIES introspection_${dependent_package})
    set(_dependent_initializers "${_dependent_initializers}    loadPackage(\"${dependent_package}\");\n")
  endif()
endforeach()

# Find other dependent packages
foreach(pkg cpp_introspection roscpp_serialization roscpp_traits rostime)
  find_package(${pkg} REQUIRED)
  list(APPEND introspection_${package}_INCLUDE_DIRS ${${pkg}_INCLUDE_DIRS})
  list(APPEND introspection_${package}_LIBRARIES ${${pkg}_LIBRARIES})
endforeach()
list(REMOVE_DUPLICATES introspection_${package}_INCLUDE_DIRS)
list(REMOVE_DUPLICATES introspection_${package}_LIBRARIES)

# Set install RPATH
list(APPEND CMAKE_INSTALL_RPATH "${CATKIN_GLOBAL_LIB_DESTINATION}/introspection")

# Iterate over all messages and generate mex
message(STATUS "Building introspection library for package ${package}")
unset(introspection_${package}_SOURCES)
unset(_message_declarations)
unset(_message_initializers)
foreach(message ${${package}_MSGS})
  message(STATUS "  adding message ${package}/${message}...")

  # Get the stripped message definition
  execute_process(
    COMMAND rosmsg show ${package}/${message}
    OUTPUT_VARIABLE ${message}_DEFINITION
    RESULT_VARIABLE ${message}_DEFINITION_RESULT
  )
  if(NOT ${${message}_DEFINITION_RESULT} EQUAL 0)
    message(SEND_ERROR "Could not get the message definition for ${package}/${message}.")
    continue()
  endif()

  string(REPLACE "\n" ";" ${message}_FIELD_PAIRS "${${message}_DEFINITION}")
  list(REMOVE_ITEM ${message}_FIELD_PAIRS "")

  # Iterate over all fields in the message
  unset(_field_includes)
  unset(_field_initializers)
  unset(_constant_initializers)
  set(field_index 0)
  foreach(field_pair ${${message}_FIELD_PAIRS})
    if(${field_pair} MATCHES "^([^\ ]+)\ ([^\ ]+)=([^\ ]+)$")
      set(constant_type ${CMAKE_MATCH_1})
      set(constant ${CMAKE_MATCH_2})
      set(constant_value ${CMAKE_MATCH_3})
      set(_constant_initializers "${_constant_initializers}    add(Constant(\"${constant}\", \"${constant_type}\", \"${constant_value}\"));\n")

    elseif(${field_pair} MATCHES "^([^\ ]+)\ ([^\ ]+)$")
      set(field_type ${CMAKE_MATCH_1})
      set(field ${CMAKE_MATCH_2})
      string(REGEX REPLACE "\\[.*\\]" "" value_type ${field_type})

      configure_file(field.cpp.in ${message}/fields/${field}.cpp @ONLY)
      set(_field_includes "${_field_includes}#include \"fields/${field}.cpp\"\n")
      set(_field_initializers "${_field_initializers}    add(FieldPtr(new fields::Field_${field}(*this)));\n")

      math(EXPR field_index "${field_index} + 1")
    endif()
  endforeach()

  configure_file(message.cpp.in ${message}/message.cpp @ONLY)

  list(APPEND introspection_${package}_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${message}/message.cpp)
  set(_message_declarations "${_message_declarations}namespace ${message} { MessagePtr Instance(); }\n")
  set(_message_initializers "${_message_initializers}    add(${message}::Instance());\n")
endforeach()

include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(${introspection_${package}_INCLUDE_DIRS})
configure_file(package.cpp.in package.cpp @ONLY)
configure_file(package.h.in ${CATKIN_DEVEL_PREFIX}/include/introspection/${package}/package.h @ONLY)
list(APPEND introspection_${package}_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/package.cpp)

add_library(introspection_${package} SHARED ${introspection_${package}_SOURCES})
target_link_libraries(introspection_${package} ${introspection_${package}_LIBRARIES})

if(DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set_target_properties(introspection_${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/introspection)
elseif(DEFINED LIBRARY_OUTPUT_PATH)
  set_target_properties(introspection_${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH}/introspection)
else()
  set_target_properties(introspection_${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

install(TARGETS introspection_${package} DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}/introspection)
install(FILES ${CATKIN_DEVEL_PREFIX}/include/introspection/${package}/package.h DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/introspection/${package})

