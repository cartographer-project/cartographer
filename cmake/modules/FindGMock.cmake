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

if(NOT TARGET standalone_gmock_main)
  include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
  set(GMOCK_PROJECT_NAME standalone_gmock_main_project)
  set(GMOCK_PROJECT_BUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/${GMOCK_PROJECT_NAME}/src/${GMOCK_PROJECT_NAME})
  set(EXTERNAL_GMOCK_LIBRARY_PATH
    ${GMOCK_PROJECT_BUILD_DIR}/googlemock/${CMAKE_STATIC_LIBRARY_PREFIX}gmock_main${CMAKE_STATIC_LIBRARY_SUFFIX})
  set(GMOCK_CMAKE_ARGS
    "-DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_GTEST:BOOL=ON -DBUILD_SHARED_LIBS:BOOL=OFF")
  if("${CMAKE_VERSION}" VERSION_GREATER 3.2)
    # BUILD_BYPRODUCTS requires CMake 3.2.
    ExternalProject_Add(
      ${GMOCK_PROJECT_NAME}
      PREFIX ${GMOCK_PROJECT_NAME}
      BUILD_IN_SOURCE 1
      URL https://github.com/google/googletest/archive/release-1.8.0.zip
      INSTALL_COMMAND ""
      CMAKE_CACHE_ARGS ${GMOCK_CMAKE_ARGS}
      BUILD_BYPRODUCTS ${EXTERNAL_GMOCK_LIBRARY_PATH}
    )
  else()
    if(CMAKE_GENERATOR MATCHES "Ninja")
      message(ERROR "This CMake version does not support BUILD_BYPRODUCTS."
        "A work-around is to use Make instead of Ninja.")
    endif()
    ExternalProject_Add(
      ${GMOCK_PROJECT_NAME}
      PREFIX ${GMOCK_PROJECT_NAME}
      BUILD_IN_SOURCE 1
      URL https://github.com/google/googletest/archive/release-1.8.0.zip
      INSTALL_COMMAND ""
      CMAKE_CACHE_ARGS ${GMOCK_CMAKE_ARGS}
    )
  endif()
  set(EXTERNAL_GTEST_INCLUDE_DIR ${GMOCK_PROJECT_BUILD_DIR}/googletest/include)
  set(EXTERNAL_GMOCK_INCLUDE_DIR ${GMOCK_PROJECT_BUILD_DIR}/googlemock/include)
  # Create directories as a work-around so they exist before compile time.
  file(MAKE_DIRECTORY ${EXTERNAL_GMOCK_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${EXTERNAL_GTEST_INCLUDE_DIR})
  add_library(standalone_gmock_main STATIC IMPORTED)
  set_target_properties(standalone_gmock_main
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
    "${EXTERNAL_GMOCK_INCLUDE_DIR};${EXTERNAL_GTEST_INCLUDE_DIR}"
  )
  set_target_properties(standalone_gmock_main
    PROPERTIES IMPORTED_LOCATION
    ${EXTERNAL_GMOCK_LIBRARY_PATH}
  )
  add_dependencies(standalone_gmock_main ${GMOCK_PROJECT_NAME})
endif()

