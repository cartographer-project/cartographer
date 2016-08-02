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

cmake_minimum_required(VERSION 2.8.3)
project(google_cartographer_msgs)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

add_message_files(
  FILES
    SubmapList.msg
    TrajectorySubmapList.msg
    SubmapEntry.msg
)

add_service_files(
  FILES
    SubmapQuery.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package()
