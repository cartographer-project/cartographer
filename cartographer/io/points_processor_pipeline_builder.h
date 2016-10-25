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

#ifndef CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_
#define CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Singleton that knows how to build a points processor pipeline out of a Lua
// configuration. All the PointsProcessor shipping with Cartographer are already
// registered with 'instance', but can register new classes with it that must
// define its name and a way to build itself out of a LuaParameterDictionary.
// See the various 'PointsProcessor's for examples.
class PointsProcessorPipelineBuilder {
 public:
  PointsProcessorPipelineBuilder(const PointsProcessorPipelineBuilder&) =
      delete;
  PointsProcessorPipelineBuilder& operator=(
      const PointsProcessorPipelineBuilder&) = delete;

  static PointsProcessorPipelineBuilder* instance();

  template <typename PointsProcessorType>
  void Register() {
    instance()->RegisterNonStatic<PointsProcessorType>();
  }

  std::vector<std::unique_ptr<PointsProcessor>> CreatePipeline(
      common::LuaParameterDictionary* dictionary) const;

 private:
  using FactoryFunction = std::function<std::unique_ptr<PointsProcessor>(
      common::LuaParameterDictionary*, PointsProcessor* next)>;

  template <typename PointsProcessorType>
  void RegisterNonStatic() {
    RegisterType(
        PointsProcessorType::kConfigurationFileActionName,
        [](common::LuaParameterDictionary* const dictionary,
           PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
          return PointsProcessorType::FromDictionary(dictionary, next);
        });
  }

  PointsProcessorPipelineBuilder();

  void RegisterType(const std::string& name, FactoryFunction factory);

  std::unordered_map<std::string, FactoryFunction> factories_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_
