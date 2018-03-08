/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_CLOUD_MAP_BUILDER_SERVER_OPTIONS_H_
#define CARTOGRAPHER_CLOUD_MAP_BUILDER_SERVER_OPTIONS_H_

#include <string>

#include "cartographer/cloud/proto/map_builder_server_options.pb.h"
#include "cartographer/common/lua_parameter_dictionary.h"

namespace cartographer {
namespace cloud {

proto::MapBuilderServerOptions CreateMapBuilderServerOptions(
    common::LuaParameterDictionary* lua_parameter_dictionary);

proto::MapBuilderServerOptions LoadMapBuilderServerOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_MAP_BUILDER_SERVER_OPTIONS_H_
