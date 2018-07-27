# Copyright 2018 The Cartographer Authors
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

cmake_minimum_required(VERSION 3.2)

if(NOT TARGET standalone_absl)
  include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
  set(ABSEIL_PROJECT_NAME abseil)
  set(ABSEIL_PROJECT_SRC_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/${ABSEIL_PROJECT_NAME}/src/${ABSEIL_PROJECT_NAME})
  set(ABSEIL_PROJECT_BUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/${ABSEIL_PROJECT_NAME}/src/${ABSEIL_PROJECT_NAME}-build)
  set(ABSEIL_INCLUDE_DIRS ${ABSEIL_PROJECT_SRC_DIR})
  set(ABSEIL_LIBRARY_PATH
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/synchronization/libabsl_synchronization.a")
  set(ABSEIL_DEPENDENT_LIBRARIES
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/algorithm/libabsl_algorithm.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/libabsl_base.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/libabsl_dynamic_annotations.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/libabsl_malloc_internal.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/libabsl_spinlock_wait.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/libabsl_throw_delegate.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/libabsl_container.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/libtest_instance_tracker_lib.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_debugging.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_examine_stack.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_failure_signal_handler.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_leak_check.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_stack_consumption.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_stacktrace.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/libabsl_symbolize.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/memory/libabsl_memory.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/meta/libabsl_meta.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/libabsl_int128.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/libabsl_numeric.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/libabsl_strings.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/time/libabsl_time.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/libabsl_any.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/libabsl_bad_any_cast.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/libabsl_bad_optional_access.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/libabsl_optional.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/libabsl_span.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/libabsl_variant.a"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/utility/libabsl_utility.a"
    )

   if(CMAKE_GENERATOR MATCHES "Ninja")
     set (ABSL_BUILD_COMMAND "ninja")
   else()
     set (ABSL_BUILD_COMMAND "make")
   endif()

  ExternalProject_Add(${ABSEIL_PROJECT_NAME}
    PREFIX ${ABSEIL_PROJECT_NAME}
    GIT_REPOSITORY   https://github.com/abseil/abseil-cpp.git
    GIT_TAG          44aa275286baf97fc13529aca547a88b180beb08
    INSTALL_COMMAND  ""
    BUILD_ALWAYS TRUE
    BUILD_COMMAND ${ABSEIL_BUILD_COMMAND}
    CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON;-DBUILD_TESTING:BOOL=OFF;-DCMAKE_BUILD_TYPE:STRING=Release"
    BUILD_BYPRODUCTS "${ABSEIL_LIBRARY_PATH};${ABSEIL_DEPENDENT_LIBRARIES}"
  )
  add_library(standalone_absl STATIC IMPORTED)
  set_target_properties(standalone_absl
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
    ${ABSEIL_INCLUDE_DIRS}
  )
  set_target_properties(standalone_absl
    PROPERTIES IMPORTED_LOCATION
    ${ABSEIL_LIBRARY_PATH}
    INTERFACE_LINK_LIBRARIES
    "${ABSEIL_DEPENDENT_LIBRARIES}"
  )
  add_dependencies(standalone_absl ${ABSEIL_PROJECT_NAME})
endif()
