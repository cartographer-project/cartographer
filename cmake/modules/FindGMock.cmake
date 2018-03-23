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

message(ERROR "!!!! my script")
if(NOT TARGET gmock_main)
    message(ERROR "!!!! my script 2")
    include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
    set(GTEST_PROJECT_NAME external-gmock)
    ExternalProject_Add(
      ${GTEST_PROJECT_NAME}
      URL https://github.com/google/googletest/archive/release-1.8.0.zip
      INSTALL_COMMAND ""
    )
    add_library(gmock_main STATIC IMPORTED)
    set_target_properties(
      gmock_main PROPERTIES IMPORTED_LOCATION
      ${CMAKE_CURRENT_BINARY_DIR}/${GTEST_PROJECT_NAME}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}gmock_main${CMAKE_STATIC_LIBRARY_SUFFIX}
    )
    # add imported include location!
    add_dependencies(gmock_main ${GTEST_PROJECT_NAME})
    #set(GMOCK_LIBRARIES gmock_main)
    #set(GMOCK_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/${GTEST_PROJECT_NAME}/include)
endif()
#include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(GMock DEFAULT_MSG GMOCK_LIBRARIES
#        GMOCK_INCLUDE_DIRS)


