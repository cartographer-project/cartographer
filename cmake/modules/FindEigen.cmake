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

find_package(Eigen3 QUIET
                    NO_MODULE
                    NO_CMAKE_PACKAGE_REGISTRY
                    NO_CMAKE_BUILDS_PATH)
if (EIGEN3_FOUND)
  message(STATUS "Found installed version of Eigen: ${Eigen3_DIR}")
else()
  # Failed to find an installed version of Eigen, repeat search allowing
  # exported build directories.
  message(STATUS "Failed to find installed Eigen CMake configuration, "
    "searching for Eigen build directories exported with CMake.")
  # Again pass NO_CMAKE_BUILDS_PATH, as we know that Eigen is exported and
  # do not want to treat projects built with the CMake GUI preferentially.
  find_package(Eigen3 QUIET
                      NO_MODULE
                      NO_CMAKE_BUILDS_PATH)
  if (EIGEN3_FOUND)
    message(STATUS "Found exported Eigen build directory: ${Eigen3_DIR}")
  endif()
endif()

if (NOT EIGEN3_FOUND)
  message(STATUS "Failed to find an installed/exported CMake configuration "
    "for Eigen, will perform search for installed Eigen components.")
  # Search user-installed locations first, so that we prefer user installs
  # to system installs where both exist.
  list(APPEND EIGEN_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
  # Additional suffixes to try appending to each search path.
  list(APPEND EIGEN_CHECK_PATH_SUFFIXES
    eigen3 # Default root directory for Eigen.
    Eigen/include/eigen3 # Windows (for C:/Program Files prefix) < 3.3
    Eigen3/include/eigen3 ) # Windows (for C:/Program Files prefix) >= 3.3

  # Search supplied hint directories first if supplied.
  find_path(EIGEN_INCLUDE_DIR
    NAMES Eigen/Core
    HINTS ${EIGEN_INCLUDE_DIR_HINTS}
    PATHS ${EIGEN_CHECK_INCLUDE_DIRS}
    PATH_SUFFIXES ${EIGEN_CHECK_PATH_SUFFIXES})

  if (EIGEN_INCLUDE_DIR AND EXISTS ${EIGEN_INCLUDE_DIR})
    set(EIGEN3_FOUND TRUE)
  else()
    message("Could not find eigen3 include directory, set EIGEN_INCLUDE_DIR to "
      "path to eigen3 include directory, e.g. /usr/local/include/eigen3.")
  endif()
endif()

# Set standard CMake FindPackage variables if found.
if (EIGEN3_FOUND)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})
endif (EIGEN_FOUND)
