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

find_path(GMOCK_INCLUDE_DIRS gmock/gmock.h
  HINTS
    ENV GMOCK_DIR
  PATH_SUFFIXES include
  PATHS
    /usr
)

# Find system-wide installed gmock.
find_library(GMOCK_LIBRARIES
  NAMES gmock_main
  HINTS
    ENV GMOCK_DIR
  PATH_SUFFIXES lib
  PATHS
    /usr
)

# Find system-wide gtest header.
find_path(GTEST_INCLUDE_DIRS gtest/gtest.h
  HINTS
    ENV GTEST_DIR
  PATH_SUFFIXES include
  PATHS
    /usr
)
list(APPEND GMOCK_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})

if(NOT GMOCK_LIBRARIES)
  # If no system-wide gmock found, then find src version.
  # Ubuntu might have this.
  find_path(GMOCK_SRC_DIR src/gmock.cc
    HINTS
      ENV GMOCK_DIR
    PATHS
      /usr/src/googletest/googlemock
      /usr/src/gmock
  )
  if(GMOCK_SRC_DIR)
    # If src version found, build it.
    add_subdirectory(${GMOCK_SRC_DIR} "${CMAKE_CURRENT_BINARY_DIR}/gmock")
    # The next line is needed for Ubuntu Trusty.
    set(GMOCK_INCLUDE_DIRS "${GMOCK_SRC_DIR}/gtest/include")
    set(GMOCK_LIBRARIES gmock_main)
  endif()
endif()

# System-wide installed gmock library might require pthreads.
find_package(Threads REQUIRED)
list(APPEND GMOCK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GMock DEFAULT_MSG GMOCK_LIBRARIES
                                  GMOCK_INCLUDE_DIRS)

