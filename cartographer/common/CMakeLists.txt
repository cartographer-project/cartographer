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

configure_file (
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/config.h)

google_test(common_blocking_queue_test
  SRCS
    blocking_queue_test.cc
)

google_test(common_configuration_files_test
  SRCS
    configuration_files_test.cc
)

google_test(common_fixed_ratio_sampler_test
  SRCS
    fixed_ratio_sampler_test.cc
)

google_test(common_lua_parameter_dictionary_test
  SRCS
    lua_parameter_dictionary_test.cc
)

google_test(common_math_test
  SRCS
    math_test.cc
)

google_test(common_rate_timer_test
  SRCS
    rate_timer_test.cc
)
