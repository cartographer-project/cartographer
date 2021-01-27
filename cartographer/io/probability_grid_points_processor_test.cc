/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/io/fake_file_writer.h"
#include "cartographer/io/points_processor_pipeline_builder.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

std::unique_ptr<PointsBatch> CreatePointsBatch() {
  auto points_batch = ::absl::make_unique<PointsBatch>();
  points_batch->origin = Eigen::Vector3f(0, 0, 0);
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 0.0f, 0.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 1.0f, 2.0f}});
  points_batch->points.push_back({Eigen::Vector3f{1.0f, 2.0f, 4.0f}});
  points_batch->points.push_back({Eigen::Vector3f{0.0f, 3.0f, 5.0f}});
  points_batch->points.push_back({Eigen::Vector3f{3.0f, 0.0f, 6.0f}});
  return points_batch;
}

::cartographer::io::FileWriterFactory CreateFakeFileWriterFactory(
    const std::string& expected_filename,
    std::shared_ptr<std::vector<char>> fake_file_writer_output) {
  return [&fake_file_writer_output,
          &expected_filename](const std::string& full_filename) {
    EXPECT_EQ(expected_filename, full_filename);
    return ::absl::make_unique<::cartographer::io::FakeFileWriter>(
        full_filename, fake_file_writer_output);
  };
}

std::vector<std::unique_ptr<::cartographer::io::PointsProcessor>>
CreatePipelineFromDictionary(
    common::LuaParameterDictionary* const pipeline_dictionary,
    const std::vector<mapping::proto::Trajectory>& trajectories,
    ::cartographer::io::FileWriterFactory file_writer_factory) {
  auto builder =
      ::absl::make_unique<::cartographer::io::PointsProcessorPipelineBuilder>();
  builder->Register(
      ProbabilityGridPointsProcessor::kConfigurationFileActionName,
      [&trajectories, file_writer_factory](
          common::LuaParameterDictionary* const dictionary,
          PointsProcessor* const next) -> std::unique_ptr<PointsProcessor> {
        return ProbabilityGridPointsProcessor::FromDictionary(
            trajectories, file_writer_factory, dictionary, next);
      });

  return builder->CreatePipeline(pipeline_dictionary);
}

std::vector<char> CreateExpectedProbabilityGrid(
    std::unique_ptr<PointsBatch> points_batch,
    common::LuaParameterDictionary* const probability_grid_options) {
  ::cartographer::mapping::ProbabilityGridRangeDataInserter2D
      range_data_inserter(
          cartographer::mapping::
              CreateProbabilityGridRangeDataInserterOptions2D(
                  probability_grid_options->GetDictionary("range_data_inserter")
                      .get()));
  mapping::ValueConversionTables conversion_tables;
  auto probability_grid = CreateProbabilityGrid(
      probability_grid_options->GetDouble("resolution"), &conversion_tables);
  range_data_inserter.Insert(
      {points_batch->origin, sensor::PointCloud(points_batch->points), {}},
      &probability_grid);

  std::vector<char> probability_grid_proto(
      probability_grid.ToProto().ByteSize());
  probability_grid.ToProto().SerializePartialToArray(
      probability_grid_proto.data(), probability_grid_proto.size());
  return probability_grid_proto;
}

std::unique_ptr<common::LuaParameterDictionary> CreateParameterDictionary() {
  auto parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          R"text(
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
          return pipeline
    )text",
          absl::make_unique<cartographer::common::DummyFileResolver>());
  return parameter_dictionary;
}

class ProbabilityGridPointsProcessorTest : public ::testing::Test {
 protected:
  ProbabilityGridPointsProcessorTest()
      : pipeline_dictionary_(CreateParameterDictionary()) {}

  void Run(const std::string& expected_filename) {
    const auto pipeline = CreatePipelineFromDictionary(
        pipeline_dictionary_.get(), dummy_trajectories_,
        CreateFakeFileWriterFactory(expected_filename,
                                    fake_file_writer_output_));
    EXPECT_TRUE(pipeline.size() > 0);

    do {
      pipeline.back()->Process(CreatePointsBatch());
    } while (pipeline.back()->Flush() ==
             cartographer::io::PointsProcessor::FlushResult::kRestartStream);
  }

  std::shared_ptr<std::vector<char>> fake_file_writer_output_ =
      std::make_shared<std::vector<char>>();
  std::unique_ptr<cartographer::common::LuaParameterDictionary>
      pipeline_dictionary_;
  const std::vector<mapping::proto::Trajectory> dummy_trajectories_;
};

TEST_F(ProbabilityGridPointsProcessorTest, WriteProto) {
  const auto expected_prob_grid_proto = CreateExpectedProbabilityGrid(
      CreatePointsBatch(),
      pipeline_dictionary_->GetArrayValuesAsDictionaries().front().get());
  Run("map.pb");
  EXPECT_THAT(*fake_file_writer_output_,
              ::testing::ContainerEq(expected_prob_grid_proto));
}

}  // namespace
}  // namespace io
}  // namespace cartographer
