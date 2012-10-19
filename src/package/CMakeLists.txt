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
execute_process(
  COMMAND rosmsg package ${package}
  OUTPUT_VARIABLE ${package}_MSGS
  RESULT_VARIABLE ${package}_MSGS_RESULT
)
if(NOT ${${package}_MSGS_RESULT} EQUAL 0)
  message(SEND_ERROR "Could not list all messages from package ${package}.")
  return()
endif()
string(REPLACE "\n" ";" ${package}_MSGS "${${package}_MSGS}")
string(REPLACE "${package}/" "" ${package}_MSGS "${${package}_MSGS}")
list(REMOVE_ITEM ${package}_MSGS "")

# Return if message list is empty
if(NOT ${package}_MSGS)
  return()
endif()

# Find and iterate through dependent packages
execute_process(
  COMMAND rospack depends1 ${package}
  OUTPUT_VARIABLE ${package}_DEPENDS
  RESULT_VARIABLE ${package}_DEPENDS_RESULT
)
if(NOT ${${package}_DEPENDS_RESULT} EQUAL 0)
  message(SEND_ERROR "Could not get package dependencies for package ${package}.")
  return()
endif()
string(REPLACE "\n" ";" ${package}_DEPENDS "${${package}_DEPENDS}")
list(REMOVE_ITEM ${package}_DEPENDS "")
list(APPEND ${package}_DEPENDS std_msgs)
list(REMOVE_ITEM ${package}_DEPENDS ${package})

unset(_dependent_declarations)
unset(_dependent_initializers)
foreach(dependent_package ${${package}_DEPENDS})
  message(STATUS "Package ${dependent_package} is required for ${package}")
  introspection_add(${dependent_package})
  list(APPEND introspection_${package}_LIBRARIES introspection_${dependent_package})
  set(_dependent_declarations "${_dependent_declarations}#include \"../${dependent_package}/package.h\"\n")
  set(_dependent_initializers "${_dependent_initializers}${dependent_package}::Load();\n")
endforeach()
list(APPEND CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/introspection")

# Find other dependent packages
find_package(ROS REQUIRED COMPONENTS roscpp_serialization roscpp_traits rostime)
list(APPEND introspection_${package}_INCLUDE_DIRS ${cpp_introspection_INCLUDE_DIRECTORIES} ${ROS_INCLUDE_DIRS})
list(APPEND introspection_${package}_LIBRARIES ${cpp_introspection_LIBRARIES} ${ROS_LIBRARIES})

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
      set(_constant_initializers "${_constant_initializers}add(Constant(\"${constant}\", \"${constant_type}\", \"${constant_value}\"));\n")

    elseif(${field_pair} MATCHES "^([^\ ]+)\ ([^\ ]+)$")
      set(field_type ${CMAKE_MATCH_1})
      set(field ${CMAKE_MATCH_2})
      string(REGEX REPLACE "\\[.*\\]" "" value_type ${field_type})

      configure_file(field.cpp.in ${message}/fields/${field}.cpp @ONLY)
      set(_field_includes "${_field_includes}#include \"fields/${field}.cpp\"\n")
      set(_field_initializers "${_field_initializers}add(FieldPtr(new fields::Field_${field}(*this)));\n")

      math(EXPR field_index "${field_index} + 1")
    endif()
  endforeach()

  configure_file(message.cpp.in ${message}/message.cpp @ONLY)

  list(APPEND introspection_${package}_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${message}/message.cpp)
  set(_message_declarations "${_message_declarations}namespace ${message} { MessagePtr Instance(); }\n")
  set(_message_initializers "${_message_initializers}add(${message}::Instance());\n")
endforeach()

include_directories(${introspection_${package}_INCLUDE_DIRS})
configure_file(package.cpp.in package.cpp @ONLY)
configure_file(package.h.in package.h @ONLY)
list(APPEND introspection_${package}_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/package.cpp)

add_library(introspection_${package} SHARED ${introspection_${package}_SOURCES})
target_link_libraries(introspection_${package} ${introspection_${package}_LIBRARIES})

  # introspection libraries are private (prevent name clashes if CMAKE_LIBRARY_OUTPUT_DIRECTORY is set)
  set_target_properties(introspection_${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS introspection_${package} DESTINATION lib/introspection)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package.h DESTINATION include/introspection/${package})
