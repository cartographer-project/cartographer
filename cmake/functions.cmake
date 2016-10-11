# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(CMakeParseArguments)

macro(_parse_arguments ARGS)
  set(OPTIONS
    USES_CERES
    USES_EIGEN
    USES_GLOG
    USES_LUA
    USES_BOOST
    USES_WEBP
    USES_GLOG
    USES_GFLAGS
  )
  set(ONE_VALUE_ARG )
  set(MULTI_VALUE_ARGS SRCS HDRS DEPENDS)
  cmake_parse_arguments(ARG
    "${OPTIONS}" "${ONE_VALUE_ARG}" "${MULTI_VALUE_ARGS}" ${ARGS})
endmacro(_parse_arguments)

macro(_common_compile_stuff VISIBILITY)
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

  if(ARG_USES_EIGEN)
    target_include_directories("${NAME}" SYSTEM ${VISIBILITY}
      "${EIGEN3_INCLUDE_DIR}")
    target_link_libraries("${NAME}" "${EIGEN3_LIBRARIES}")
  endif()

  if(ARG_USES_CERES)
    target_include_directories("${NAME}" SYSTEM ${VISIBILITY}
      "${CERES_INCLUDE_DIRS}")
    target_link_libraries("${NAME}" "${CERES_LIBRARIES}")
  endif()

  if(ARG_USES_LUA)
    target_include_directories("${NAME}" SYSTEM ${VISIBILITY}
      "${LUA_INCLUDE_DIR}")
    target_link_libraries("${NAME}" "${LUA_LIBRARIES}")
  endif()

  if(ARG_USES_BOOST)
    target_include_directories("${NAME}" SYSTEM ${VISIBILITY}
      "{Boost_INCLUDE_DIRS}")
    target_link_libraries("${NAME}" "${Boost_LIBRARIES}")
  endif()

  if(ARG_USES_WEBP)
    target_link_libraries("${NAME}" webp)
  endif()

  # We rely on Ceres to find glog and gflags for us.
  if(ARG_USES_GLOG)
    target_link_libraries("${NAME}" glog)
  endif()

  if(ARG_USES_GFLAGS)
    target_link_libraries("${NAME}" gflags)
  endif()

  # Add the binary directory first, so that port.h is included after it has
  # been generated.
  target_include_directories("${NAME}" ${VISIBILITY} "${CMAKE_BINARY_DIR}")
  target_include_directories("${NAME}" ${VISIBILITY} "${CMAKE_SOURCE_DIR}")

  foreach(DEPENDENCY ${ARG_DEPENDS})
    target_link_libraries(${NAME} ${DEPENDENCY})
  endforeach()

  # Figure out where to install the header. The logic goes like this: either
  # the header is in the current binary directory (i.e. generated, like port.h)
  # or in the current source directory - a regular header. It could also
  # already be absolute (i.e. generated through a google_proto_library rule).
  # In all cases we want to install the right header under the right subtree,
  # e.g. foo/bar/baz.h.in will be installed from the binary directory as
  # 'include/foo/bar/baz.h'.
  foreach(HDR ${ARG_HDRS})
    if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${HDR})
      set(ABS_FIL "${CMAKE_CURRENT_BINARY_DIR}/${HDR}")
      file(RELATIVE_PATH REL_FIL ${CMAKE_BINARY_DIR} ${ABS_FIL})
    elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${HDR})
      set(ABS_FIL "${CMAKE_CURRENT_SOURCE_DIR}/${HDR}")
      file(RELATIVE_PATH REL_FIL ${CMAKE_SOURCE_DIR} ${ABS_FIL})
    else()
      set(ABS_FIL "${HDR}")
      file(RELATIVE_PATH REL_FIL ${CMAKE_BINARY_DIR} ${ABS_FIL})
    endif()
    get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
    install(
      FILES
        ${ABS_FIL}
      DESTINATION
        include/${INSTALL_DIR}
    )
  endforeach()
endmacro(_common_compile_stuff)

# Create a static library out of other static libraries by running a GNU ar
# script over the dependencies.
function(google_combined_library NAME)
  set(MULTI_VALUE_ARGS SRCS)
  cmake_parse_arguments(ARG "" "" "${MULTI_VALUE_ARGS}" ${ARGN})

  # Cmake requires a source file for each library, so we create a dummy file
  # that is empty. Its creation depends on all libraries we want to include in
  # our combined static library, i.e. it will be touched whenever any of them
  # change, which means that our combined library is regenerated.
  set(DUMMY_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/${NAME}_dummy.cc)
  add_custom_command(
    OUTPUT  ${DUMMY_SOURCE}
    COMMAND cmake -E touch ${DUMMY_SOURCE}
    DEPENDS ${ARG_SRCS}
  )

  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
           "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${CMAKE_BINARY_DIR} -I
      ${CMAKE_BINARY_DIR} ${REWRITTEN_PROTO}
    DEPENDS ${REWRITTEN_PROTO}
    COMMENT "Running C++ protocol buffer compiler on ${FIL}"
    VERBATIM
  )

  # Just a dummy library, we will overwrite its output directly after again
  # with its POST_BUILD step.
  google_library(${NAME}
    SRCS ${DUMMY_SOURCE}
    DEPENDS ${ARG_SRCS}
  )

  get_property(OUTPUT_FILE TARGET ${NAME} PROPERTY LOCATION)

  # We will delete the static lib generated by the last call to
  # 'google_library' and recreate it using a GNU ar script that combines the
  # SRCS into the NAME.
  # TODO(hrapp): this is probably not very portable, but should work fine on
  # Linux.
  set(AR_SCRIPT "")
  set(AR_SCRIPT "CREATE ${OUTPUT_FILE}\n")
  foreach(SRC ${ARG_SRCS})
    get_property(STATIC_LIBRARY_FILE TARGET ${SRC} PROPERTY LOCATION)
    set(AR_SCRIPT "${AR_SCRIPT}ADDLIB ${STATIC_LIBRARY_FILE}\n")
  endforeach()
  set(AR_SCRIPT "${AR_SCRIPT}SAVE\nEND\n")
  set(AR_SCRIPT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${NAME}_ar.script")
  file(WRITE ${AR_SCRIPT_FILE} ${AR_SCRIPT})

  add_custom_command(TARGET ${NAME} POST_BUILD
    COMMAND rm ${OUTPUT_FILE}
    COMMAND ${CMAKE_AR}
    ARGS -M < ${AR_SCRIPT_FILE}
    COMMENT "Recombining static libraries into ${NAME}."
  )
endfunction()

# Create a variable 'VAR_NAME'='FLAG'. If VAR_NAME is already set, FLAG is
# appended.
function(google_add_flag VAR_NAME FLAG)
  if (${VAR_NAME})
    set(${VAR_NAME} "${${VAR_NAME}} ${FLAG}" PARENT_SCOPE)
  else()
    set(${VAR_NAME} "${FLAG}" PARENT_SCOPE)
  endif()
endfunction()

function(google_test NAME)
  _parse_arguments("${ARGN}")

  add_executable(${NAME}
    ${ARG_SRCS} ${ARG_HDRS}
  )

  _common_compile_stuff("PRIVATE")

  # Make sure that gmock always includes the correct gtest/gtest.h.
  target_include_directories("${NAME}" SYSTEM PRIVATE
    "${GMOCK_SRC_DIR}/gtest/include")
  target_link_libraries("${NAME}" gmock_main)

  add_test(${NAME} ${NAME})
endfunction()

function(google_binary NAME)
  _parse_arguments("${ARGN}")

  add_executable(${NAME}
    ${ARG_SRCS} ${ARG_HDRS}
  )

  _common_compile_stuff("PRIVATE")

  install(TARGETS "${NAME}" RUNTIME DESTINATION bin)
endfunction()

function(google_library NAME)
  _parse_arguments("${ARGN}")

  add_library(${NAME}
    STATIC
    ${ARG_SRCS} ${ARG_HDRS}
  )

  # Update the global variable that contains all static libraries.
  SET(ALL_LIBRARIES "${ALL_LIBRARIES};${NAME}" CACHE INTERNAL "ALL_LIBRARIES")

  # This is needed for header only libraries. While they do not really mean
  # anything for cmake, they are useful to make dependencies explicit.
  set_target_properties(${NAME} PROPERTIES LINKER_LANGUAGE CXX)

  _common_compile_stuff("PUBLIC")
endfunction()

function(google_proto_library NAME)
  _parse_arguments("${ARGN}")

  set(PROTO_SRCS)
  set(PROTO_HDRS)
  foreach(FIL ${ARG_SRCS})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(FIL_NAME ${FIL} NAME)

    list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
    list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
             "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out  ${CMAKE_BINARY_DIR} -I
        ${CMAKE_SOURCE_DIR} ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${FIL}"
      VERBATIM
    )
  endforeach()

  set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
    PROPERTIES GENERATED TRUE)

  google_library("${NAME}"
      SRCS "${PROTO_SRCS}"
      HDRS "${PROTO_HDRS}"
      DEPENDS "${ARG_DEPENDS}"
  )

  target_include_directories("${NAME}" SYSTEM "PUBLIC"
    "${PROTOBUF_INCLUDE_DIR}")
  # TODO(hrapp): This should not explicityly list pthread and use
  # PROTOBUF_LIBRARIES, but that failed on first try.
  target_link_libraries("${NAME}" "${PROTOBUF_LIBRARY}" pthread)
endfunction()
