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
#include "cartographer/mapping/2d/range_data_inserter_2d.h"
#include "cartographer/common/port.h"
#include "cartographer/io/fake_file_writer.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

std::unique_ptr<common::LuaParameterDictionary> CreateParameterDictionary() {
  auto parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          R"text(
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
              output_type = "pb", 
              filename = "map" 
            }
          } 
        } 
        return options
    )text",
          common::make_unique<cartographer::common::DummyFileResolver>());
  return parameter_dictionary;
}

std::unique_ptr<PointsBatch> CreatePointsBatch() {
  auto points_batch = cartographer::common::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.emplace_back(0.0f, 0.0f, 0.0f);
  points_batch->points.emplace_back(0.0f, 1.0f, 2.0f);
  points_batch->points.emplace_back(1.0f, 2.0f, 4.0f);
  points_batch->points.emplace_back(0.0f, 3.0f, 5.0f);
  points_batch->points.emplace_back(3.0f, 0.0f, 6.0f);
  return points_batch;
}

::cartographer::io::FileWriterFactory CreateFakeFileWriterFactory(
    const std::string& expected_filename,
    std::shared_ptr<std::string> fake_file_writer_output) {
  return [&fake_file_writer_output,
          &expected_filename](const std::string& full_filename) {
    EXPECT_EQ(expected_filename, full_filename);
    return ::cartographer::common::make_unique<
        ::cartographer::io::FakeFileWriter>(full_filename,
                                            fake_file_writer_output);
  };
}

std::vector<std::unique_ptr<::cartographer::io::PointsProcessor>>
CreatePipelineFromDictionary(
    std::unique_ptr<common::LuaParameterDictionary> pipeline_dictionary,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    ::cartographer::io::FileWriterFactory file_writer_factory) {
  auto builder = ::cartographer::common::make_unique<
      ::cartographer::io::PointsProcessorPipelineBuilder>();
  builder->Register(
      ProbabilityGridPointsProcessor::kConfigurationFileActionName,
      [&trajectories, file_writer_factory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return ProbabilityGridPointsProcessor::FromDictionary(
            trajectories, file_writer_factory, dictionary, next);
      });

  return builder->CreatePipeline(pipeline_dictionary.get());
}

std::string CreateExpectedProbabilityGrid(
    std::unique_ptr<PointsBatch> points_batch,
    common::LuaParameterDictionary* const probability_grid_options) {
  ::cartographer::mapping::RangeDataInserter2D range_data_inserter(
      cartographer::mapping::CreateRangeDataInserterOptions2D(
          probability_grid_options->GetDictionary("range_data_inserter")
              .get()));
  auto probability_grid =
      CreateProbabilityGrid(probability_grid_options->GetDouble("resolution"));
  range_data_inserter.Insert({points_batch->origin, points_batch->points, {}},
                             &probability_grid);

  std::string expected_probability_grid_proto_string;
  probability_grid.ToProto().SerializeToString(
      &expected_probability_grid_proto_string);
  return expected_probability_grid_proto_string;
}

TEST(ProbabilityGridPointsProcessor, WriteProto) {
  auto parameter_dictionary = CreateParameterDictionary();
  auto pipeline_dict = parameter_dictionary->GetDictionary("pipeline");
  std::vector<std::unique_ptr<common::LuaParameterDictionary>> configurations =
      pipeline_dict->GetArrayValuesAsDictionaries();
  const auto& write_prob_options = configurations.front();
  EXPECT_EQ(write_prob_options->GetString("action"), "write_probability_grid");

  const std::vector<mapping::proto::Trajectory> dummy_trajectories;
  auto points_batch = CreatePointsBatch();

  auto expected_prob_grid_proto_string = CreateExpectedProbabilityGrid(
      std::move(points_batch), write_prob_options.get());

  auto fake_file_writer_output = std::make_shared<std::string>();
  auto pipeline = CreatePipelineFromDictionary(
      std::move(pipeline_dict), dummy_trajectories,
      CreateFakeFileWriterFactory(
          write_prob_options->GetString("filename") + ".pb",
          fake_file_writer_output));
  EXPECT_TRUE(pipeline.size() > 0);

  do {
    pipeline.back()->Process(CreatePointsBatch());
  } while (pipeline.back()->Flush() ==
           cartographer::io::PointsProcessor::FlushResult::kRestartStream);

  EXPECT_TRUE(*fake_file_writer_output ==
              expected_prob_grid_proto_string);
}

}  // namespace
}  // namespace io
}  // namespace cartographer
