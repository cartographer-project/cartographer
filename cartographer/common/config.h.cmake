/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_CONFIG_H_
#define CARTOGRAPHER_COMMON_CONFIG_H_

namespace cartographer {
namespace common {

constexpr char kConfigurationFilesDirectory[] =
    "@CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY@";
constexpr char kSourceDirectory[] = "@CMAKE_SOURCE_DIR@";

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_CONFIG_H_
