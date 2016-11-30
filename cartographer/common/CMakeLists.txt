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

add_subdirectory("proto")

configure_file (
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/config.h)

google_library(common_blocking_queue
  USES_GLOG
  HDRS
    blocking_queue.h
  DEPENDS
    common_mutex
    common_port
    common_time
)

google_library(common_ceres_solver_options
  USES_CERES
  SRCS
    ceres_solver_options.cc
  HDRS
    ceres_solver_options.h
  DEPENDS
    common_lua_parameter_dictionary
    common_proto_ceres_solver_options
)

google_library(common_config
  HDRS
    config.h
)

google_library(common_configuration_file_resolver
  USES_GLOG
  SRCS
    configuration_file_resolver.cc
  HDRS
    configuration_file_resolver.h
  DEPENDS
    common_config
    common_lua_parameter_dictionary
    common_port
)

google_library(common_fixed_ratio_sampler
  SRCS
    fixed_ratio_sampler.cc
  HDRS
    fixed_ratio_sampler.h
  DEPENDS
    common_port
)

google_library(common_histogram
  USES_GLOG
  SRCS
    histogram.cc
  HDRS
    histogram.h
  DEPENDS
    common_port
)

google_library(common_interval
  HDRS
    interval.h
)

google_library(common_lua
  USES_LUA
  HDRS
    lua.h
)

google_library(common_lua_parameter_dictionary
  USES_GLOG
  SRCS
    lua_parameter_dictionary.cc
  HDRS
    lua_parameter_dictionary.h
  DEPENDS
    common_lua
    common_port
)

google_library(common_lua_parameter_dictionary_test_helpers
  USES_GLOG
  HDRS
    lua_parameter_dictionary_test_helpers.h
  DEPENDS
    common_lua_parameter_dictionary
    common_make_unique
    common_port
)

google_library(common_make_unique
  HDRS
    make_unique.h
)

google_library(common_math
  USES_CERES
  USES_EIGEN
  HDRS
    math.h
  DEPENDS
    common_port
)

google_library(common_mutex
  HDRS
    mutex.h
  DEPENDS
    common_time
)

google_library(common_port
  USES_BOOST
  HDRS
    port.h
)

google_library(common_rate_timer
  HDRS
    rate_timer.h
  DEPENDS
    common_math
    common_port
    common_time
)

google_library(common_thread_pool
  USES_GLOG
  SRCS
    thread_pool.cc
  HDRS
    thread_pool.h
  DEPENDS
    common_mutex
)

google_library(common_time
  SRCS
    time.cc
  HDRS
    time.h
  DEPENDS
    common_port
)

google_test(common_blocking_queue_test
  SRCS
    blocking_queue_test.cc
  DEPENDS
    common_blocking_queue
    common_make_unique
    common_time
)

google_test(common_configuration_files_test
  SRCS
    configuration_files_test.cc
  DEPENDS
    common_config
    common_configuration_file_resolver
    common_lua_parameter_dictionary
    mapping_map_builder
)

google_test(common_fixed_ratio_sampler_test
  SRCS
    fixed_ratio_sampler_test.cc
  DEPENDS
    common_fixed_ratio_sampler
)

google_test(common_lua_parameter_dictionary_test
  SRCS
    lua_parameter_dictionary_test.cc
  DEPENDS
    common_lua_parameter_dictionary
    common_lua_parameter_dictionary_test_helpers
    common_make_unique
)

google_test(common_math_test
  SRCS
    math_test.cc
  DEPENDS
    common_math
)

google_test(common_rate_timer_test
  SRCS
    rate_timer_test.cc
  DEPENDS
    common_rate_timer
)
