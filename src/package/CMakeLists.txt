if(NOT DEFINED package)
  message(SEND_FATAL "You have to define variable package to determine the package for which introspection library is going to be built.")
  return()
endif(NOT DEFINED package)
message(STATUS "Building introspection library for package ${package}")
set(CMAKE_BUILD_TYPE MinSizeRel)

# Find dependent packages
find_package(roscpp_serialization REQUIRED)
find_package(roscpp_traits REQUIRED)
find_package(rostime REQUIRED)
set(${package}_INCLUDE_DIRS ${${package}_INCLUDE_DIRS} ${roscpp_serialization_INCLUDE_DIRS} ${roscpp_traits_INCLUDE_DIRS} ${rostime_INCLUDE_DIRS})
set(${package}_LIBRARIES ${${package}_LIBRARIES} ${roscpp_serialization_LIBRARIES} ${roscpp_traits_LIBRARIES} ${rostime_LIBRARIES})

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

# Iterate over all messages and generate mex
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
  set(field_index 0)
  foreach(field_pair ${${message}_FIELD_PAIRS})
    if(${field_pair} MATCHES "^([^\ ]+)\ ([^\ ]+)$")
      set(field ${CMAKE_MATCH_2})
      set(field_type ${CMAKE_MATCH_1})
      string(REGEX REPLACE "\\[.*\\]" "" value_type ${field_type})

      configure_file(field.cpp.in ${message}/fields/${field}.cpp @ONLY)
      set(_field_includes "${_field_includes}#include \"fields/${field}.cpp\"\n")
      set(_field_initializers "${_field_initializers}add(FieldPtr(new fields::field_${field}(*this)));\n")

      math(EXPR field_index "${field_index} + 1")
    endif()
  endforeach()

  configure_file(message.cpp.in ${message}/message.cpp @ONLY)

  list(APPEND introspection_${package}_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/${message}/message.cpp)
  set(_message_declarations "${_message_declarations}namespace ${message} { MessagePtr createIntrospection(const Package&); }\n")
  set(_messags_initializers "${_messags_initializers}add(${message}::createIntrospection(*this));\n")
endforeach()

include_directories(${${package}_INCLUDE_DIRS})
configure_file(package.cpp.in package.cpp @ONLY)
list(APPEND introspection_${package}_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/package.cpp)

add_library(introspection_${package} SHARED ${introspection_${package}_SOURCES})
set_target_properties(introspection_${package} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib/introspection")
target_link_libraries(introspection_${package} ${${package}_LIBRARIES})
