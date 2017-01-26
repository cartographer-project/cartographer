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
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace io {

// Builder to create a points processor pipeline out of a Lua configuration.
// You can register all built-in PointsProcessors using
// 'RegisterBuiltInPointsProcessors'. Non-built-in PointsProcessors must define
// a name and a factory method for building itself from a
// LuaParameterDictionary. See the various built-in PointsProcessors for
// examples.
class PointsProcessorPipelineBuilder {
 public:
  using FactoryFunction = std::function<std::unique_ptr<PointsProcessor>(
      common::LuaParameterDictionary*, PointsProcessor* next)>;

  PointsProcessorPipelineBuilder();

  PointsProcessorPipelineBuilder(const PointsProcessorPipelineBuilder&) =
      delete;
  PointsProcessorPipelineBuilder& operator=(
      const PointsProcessorPipelineBuilder&) = delete;

  // Register a new PointsProcessor type uniquly identified by 'name' which will
  // be created using 'factory'.
  void Register(const std::string& name, FactoryFunction factory);

  std::vector<std::unique_ptr<PointsProcessor>> CreatePipeline(
      common::LuaParameterDictionary* dictionary) const;

 private:
  std::unordered_map<std::string, FactoryFunction> factories_;
};

// Register all 'PointsProcessor' that ship with Cartographer with this
// 'builder'.
void RegisterBuiltInPointsProcessors(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    PointsProcessorPipelineBuilder* builder);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_PROCESSOR_PIPELINE_BUILDER_H_
