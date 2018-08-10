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

include(${CMAKE_CURRENT_LIST_DIR}/FetchContent.cmake)
FetchContent_Declare(abseil
  GIT_REPOSITORY   https://github.com/abseil/abseil-cpp.git
  GIT_TAG          44aa275286baf97fc13529aca547a88b180beb08
)

function(_populate_add_abseil)
  FetchContent_Populate(abseil)
  set(BUILD_TESTING OFF)
  add_subdirectory(${abseil_SOURCE_DIR} ${abseil_BINARY_DIR})
  add_library(standalone_absl INTERFACE IMPORTED GLOBAL)
  target_link_libraries(standalone_absl INTERFACE absl_synchronization)
endfunction()

FetchContent_GetProperties(abseil)
if(NOT abseil_POPULATED)
  _populate_add_abseil()
endif()
