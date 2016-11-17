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

#include "cartographer/io/points_processor_pipeline_builder.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/counting_points_processor.h"
#include "cartographer/io/fixed_ratio_sampling_points_processor.h"
#include "cartographer/io/min_max_range_filtering_points_processor.h"
#include "cartographer/io/null_points_processor.h"
#include "cartographer/io/outlier_removing_points_processor.h"
#include "cartographer/io/pcd_writing_points_processor.h"
#include "cartographer/io/ply_writing_points_processor.h"
#include "cartographer/io/xray_points_processor.h"
#include "cartographer/io/xyz_writing_points_processor.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace io {

template <typename PointsProcessorType>
void RegisterPlainPointsProcessor(
    PointsProcessorPipelineBuilder* const builder) {
  builder->Register(
      PointsProcessorType::kConfigurationFileActionName,
      [](common::LuaParameterDictionary* const dictionary,
         PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return PointsProcessorType::FromDictionary(dictionary, next);
      });
}

void RegisterBuiltInPointsProcessors(
    const mapping::proto::Trajectory& trajectory,
    PointsProcessorPipelineBuilder* builder) {
  RegisterPlainPointsProcessor<CountingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<FixedRatioSamplingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<MinMaxRangeFiteringPointsProcessor>(builder);
  RegisterPlainPointsProcessor<OutlierRemovingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<PcdWritingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<PlyWritingPointsProcessor>(builder);
  RegisterPlainPointsProcessor<XyzWriterPointsProcessor>(builder);

  builder->Register(
      XRayPointsProcessor::kConfigurationFileActionName,
      [&trajectory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return XRayPointsProcessor::FromDictionary(trajectory, dictionary,
                                                   next);
      });
}

void PointsProcessorPipelineBuilder::Register(const std::string& name,
                                              FactoryFunction factory) {
  CHECK(factories_.count(name) == 0) << "A points processor with named '"
                                     << name
                                     << "' has already been registered.";
  factories_[name] = factory;
}

PointsProcessorPipelineBuilder::PointsProcessorPipelineBuilder() {}

std::vector<std::unique_ptr<PointsProcessor>>
PointsProcessorPipelineBuilder::CreatePipeline(
    common::LuaParameterDictionary* const dictionary) const {
  std::vector<std::unique_ptr<PointsProcessor>> pipeline;
  // The last consumer in the pipeline must exist, so that the one created after
  // it (and being before it in the pipeline) has a valid 'next' to point to.
  // The last consumer will just drop all points.
  pipeline.emplace_back(common::make_unique<NullPointsProcessor>());

  std::vector<std::unique_ptr<common::LuaParameterDictionary>> configurations =
      dictionary->GetArrayValuesAsDictionaries();

  // We construct the pipeline starting at the back.
  for (auto it = configurations.rbegin(); it != configurations.rend(); it++) {
    const string action = (*it)->GetString("action");
    auto factory_it = factories_.find(action);
    CHECK(factory_it != factories_.end())
        << "Unknown action '" << action
        << "'. Did you register the correspoinding PointsProcessor?";
    pipeline.push_back(factory_it->second(it->get(), pipeline.back().get()));
  }
  return pipeline;
}

}  // namespace io
}  // namespace cartographer
