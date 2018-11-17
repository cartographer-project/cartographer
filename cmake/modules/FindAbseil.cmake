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
  set(prefix ${CMAKE_STATIC_LIBRARY_PREFIX})
  set(suffix ${CMAKE_STATIC_LIBRARY_SUFFIX})
  include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
  set(ABSEIL_PROJECT_NAME abseil)
  set(ABSEIL_PROJECT_SRC_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/${ABSEIL_PROJECT_NAME}/src/${ABSEIL_PROJECT_NAME})
  set(ABSEIL_PROJECT_BUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/${ABSEIL_PROJECT_NAME}/src/${ABSEIL_PROJECT_NAME}-build)
  set(ABSEIL_INCLUDE_DIRS ${ABSEIL_PROJECT_SRC_DIR})
  set(ABSEIL_LIBRARY_PATH
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/synchronization/${prefix}absl_synchronization${suffix}")
  set(ABSEIL_DEPENDENT_LIBRARIES
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_symbolize${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/time/${prefix}absl_time${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${prefix}str_format_internal${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${prefix}str_format_extension_internal${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${prefix}absl_str_format${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${prefix}absl_strings${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/hash/${prefix}absl_hash${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/algorithm/${prefix}absl_algorithm${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${prefix}absl_base${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${prefix}absl_dynamic_annotations${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${prefix}absl_internal_malloc_internal${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${prefix}absl_internal_spinlock_wait${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${prefix}absl_internal_throw_delegate${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/${prefix}absl_container${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/${prefix}test_instance_tracker_lib${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_debugging${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_examine_stack${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_failure_signal_handler${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_leak_check${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_stack_consumption${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${prefix}absl_stacktrace${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/memory/${prefix}absl_memory${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/meta/${prefix}absl_meta${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/${prefix}absl_int128${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/${prefix}absl_numeric${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/synchronization/${prefix}absl_synchronization${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${prefix}absl_any${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${prefix}absl_bad_any_cast${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${prefix}absl_bad_optional_access${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${prefix}absl_optional${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${prefix}absl_span${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${prefix}absl_variant${suffix}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/utility/${prefix}absl_utility${suffix}"
  )
  ExternalProject_Add(${ABSEIL_PROJECT_NAME}
    PREFIX ${ABSEIL_PROJECT_NAME}
    GIT_REPOSITORY   https://github.com/abseil/abseil-cpp.git
    GIT_TAG          7b46e1d31a6b08b1c6da2a13e7b151a20446fa07
    INSTALL_COMMAND  ""
    BUILD_COMMAND    ${CMAKE_COMMAND} --build "${ABSEIL_PROJECT_BUILD_DIR}"
    CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON;-DBUILD_TESTING:BOOL=OFF;-DCMAKE_BUILD_TYPE:STRING=Release"
    BUILD_BYPRODUCTS "${ABSEIL_LIBRARY_PATH};${ABSEIL_DEPENDENT_LIBRARIES}"
  )
  add_library(standalone_absl STATIC IMPORTED GLOBAL)
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
  if(MSVC)
    # /wd4005  macro-redefinition
    # /wd4068  unknown pragma
    # /wd4244  conversion from 'type1' to 'type2'
    # /wd4267  conversion from 'size_t' to 'type2'
    # /wd4800  force value to bool 'true' or 'false' (performance warning)
    target_compile_options(standalone_absl INTERFACE /wd4005 /wd4068 /wd4244 /wd4267 /wd4800)
    target_compile_definitions(standalone_absl INTERFACE -DNOMINMAX -DWIN32_LEAN_AND_MEAN=1 -D_CRT_SECURE_NO_WARNINGS)
  endif()
  add_dependencies(standalone_absl ${ABSEIL_PROJECT_NAME})
  unset(prefix)
  unset(suffix)
endif()
