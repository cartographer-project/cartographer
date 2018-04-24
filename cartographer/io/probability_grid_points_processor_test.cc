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

#include "cartographer/io/probability_grid_points_processor.h"
#include <string>
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/range_data_inserter_2d.h"

#include "cartographer/common/port.h"

#include "cartographer/io/fake_file_writer.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

TEST(ProbabilityGridPointsProcessor, WriteProto) {
  auto parameter_dictionary = common::MakeDictionary(R"text(
        options = { 
          tracking_frame = "base_link", 
          pipeline = { 
            { 
              action = "write_probability_grid", 
              resolution = 0.05, 
              range_data_inserter = { 
                insert_free_space = true, 
                hit_probability = 0.55, 
                miss_probability = 0.49, 
              }, 
              draw_trajectories = false, 
              output_type = "pbstream", 
              filename = "map" 
            }
          } 
        } 
        return options
    )text");

  const auto tracking_frame = parameter_dictionary->GetString("tracking_frame");
  const auto pipeline_dict = parameter_dictionary->GetDictionary("pipeline");
  std::vector<std::unique_ptr<common::LuaParameterDictionary>> configurations =
      pipeline_dict->GetArrayValuesAsDictionaries();
  const auto& write_prob_options = configurations.front();
  EXPECT_EQ( write_prob_options->GetString("action"), "write_probability_grid");

  const std::vector<mapping::proto::Trajectory> dummy_trajectories;
  auto points_batch = cartographer::common::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.emplace_back(0.0f, 0.0f, 0.0f);
  points_batch->points.emplace_back(0.0f, 1.0f, 2.0f);
  points_batch->points.emplace_back(1.0f, 2.0f, 4.0f);
  points_batch->points.emplace_back(0.0f, 3.0f, 5.0f);
  points_batch->points.emplace_back(3.0f, 0.0f, 6.0f);

  ::cartographer::mapping::RangeDataInserter2D range_data_inserter(
      cartographer::mapping::CreateRangeDataInserterOptions2D(
          write_prob_options->GetDictionary("range_data_inserter").get()));
  auto probability_grid =
      CreateProbabilityGrid(write_prob_options->GetDouble("resolution"));
  range_data_inserter.Insert({points_batch->origin, points_batch->points, {}},
                             &probability_grid);

  auto points_batch_copy =
      cartographer::common::make_unique<PointsBatch>(*points_batch);
  auto builder = ::cartographer::common::make_unique<
      ::cartographer::io::PointsProcessorPipelineBuilder>();

    auto expected_filename = write_prob_options->GetString("filename") + ".proto";
  auto fake_file_writer_output = std::make_shared<std::string>();
  const auto file_writer_factory = [&fake_file_writer_output, &expected_filename
  ](const std::string& full_filename) {
    EXPECT_EQ(expected_filename, full_filename);
    return ::cartographer::common::make_unique<
        ::cartographer::io::FakeFileWriter>(full_filename,
                                            fake_file_writer_output);
  };

  builder->Register(
      ProbabilityGridPointsProcessor::kConfigurationFileActionName,
      [&dummy_trajectories, file_writer_factory, &write_prob_options](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return ProbabilityGridPointsProcessor::FromDictionary(
            dummy_trajectories, file_writer_factory, write_prob_options.get(),
            next);
      });

  auto points_processors = builder->CreatePipeline(write_prob_options.get());
  ASSERT_TRUE(points_processors.size() > 0);
  points_processors.front()->Process(std::move(points_batch_copy));
  points_processors.front()->Flush();

  std::string expected_probability_grid_proto_string;
  probability_grid.ToProto().SerializeToString(
      &expected_probability_grid_proto_string);

  LOG(INFO) << "is: " << *fake_file_writer_output << " expected: " << expected_probability_grid_proto_string;
  ASSERT_EQ(*fake_file_writer_output, expected_probability_grid_proto_string);
}

}  // namespace
}  // namespace io
}  // namespace cartographer
