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

# TODO(gaschler): Clean up, rename script.
if(NOT TARGET standalone_gmock_main)
    include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
    set(GTEST_PROJECT_NAME external-gmock)
    set(EXTERNAL_GMOCK_LIBRARY_PATH ${CMAKE_CURRENT_BINARY_DIR}/${GTEST_PROJECT_NAME}/src/${GTEST_PROJECT_NAME}/googlemock/${CMAKE_STATIC_LIBRARY_PREFIX}gmock_main${CMAKE_STATIC_LIBRARY_SUFFIX})
    ExternalProject_Add(
      ${GTEST_PROJECT_NAME}
    PREFIX ${GTEST_PROJECT_NAME}
    BUILD_IN_SOURCE 1
      URL https://github.com/google/googletest/archive/release-1.8.0.zip
      INSTALL_COMMAND ""
      BUILD_BYPRODUCTS ${EXTERNAL_GMOCK_LIBRARY_PATH}
    CMAKE_CACHE_ARGS
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DBUILD_GTEST:BOOL=ON
            -DBUILD_SHARED_LIBS:BOOL=OFF
    )
    set(EXTERNAL_GTEST_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${GTEST_PROJECT_NAME}/src/${GTEST_PROJECT_NAME}/googletest/include")
    set(EXTERNAL_GMOCK_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${GTEST_PROJECT_NAME}/src/${GTEST_PROJECT_NAME}/googlemock/include")
        # Work-around the directory will only exist at compile time
    file(MAKE_DIRECTORY ${EXTERNAL_GMOCK_INCLUDE_DIR})
    file(MAKE_DIRECTORY ${EXTERNAL_GTEST_INCLUDE_DIR})
    add_library(standalone_gmock_main STATIC IMPORTED)
    set_target_properties(standalone_gmock_main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
            "${EXTERNAL_GMOCK_INCLUDE_DIR};${EXTERNAL_GTEST_INCLUDE_DIR}"
    )
    set_target_properties(standalone_gmock_main PROPERTIES IMPORTED_LOCATION
      ${EXTERNAL_GMOCK_LIBRARY_PATH}
    )
    add_dependencies(standalone_gmock_main ${GTEST_PROJECT_NAME})
endif()

