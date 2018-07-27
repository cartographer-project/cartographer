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

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_TESTING_HANDLER_TEST_H
#define CARTOGRAPHER_CLOUD_INTERNAL_TESTING_HANDLER_TEST_H

#include "absl/memory/memory.h"
#include "async_grpc/testing/rpc_handler_test_server.h"
#include "cartographer/mapping/internal/testing/mock_map_builder.h"
#include "cartographer/mapping/internal/testing/mock_pose_graph.h"
#include "gtest/gtest.h"
#include "mock_local_trajectory_uploader.h"
#include "mock_map_builder_context.h"

namespace cartographer {
namespace cloud {
namespace testing {

using ::testing::Return;
using ::testing::Test;

template <typename HandlerConcept, typename HandlerType>
class HandlerTest : public Test {
 public:
  void SetUp() override {
    test_server_ = absl::make_unique<
        async_grpc::testing::RpcHandlerTestServer<HandlerConcept, HandlerType>>(
        absl::make_unique<MockMapBuilderContext>());
    mock_map_builder_context_ =
        test_server_
            ->template GetUnsynchronizedContext<MockMapBuilderContext>();
    mock_local_trajectory_uploader_ =
        absl::make_unique<MockLocalTrajectoryUploader>();
    mock_map_builder_ = absl::make_unique<mapping::testing::MockMapBuilder>();
    mock_pose_graph_ = absl::make_unique<mapping::testing::MockPoseGraph>();

    EXPECT_CALL(*mock_map_builder_context_, map_builder())
        .Times(::testing::AnyNumber())
        .WillRepeatedly(::testing::ReturnPointee(mock_map_builder_.get()));
    EXPECT_CALL(*mock_map_builder_, pose_graph())
        .Times(::testing::AnyNumber())
        .WillRepeatedly(Return(mock_pose_graph_.get()));
  }

  void SetNoLocalTrajectoryUploader() {
    EXPECT_CALL(*mock_map_builder_context_, local_trajectory_uploader())
        .WillOnce(Return(nullptr));
  }

  void SetMockLocalTrajectoryUploader() {
    EXPECT_CALL(*mock_map_builder_context_, local_trajectory_uploader())
        .WillRepeatedly(Return(mock_local_trajectory_uploader_.get()));
  }

 protected:
  std::unique_ptr<
      async_grpc::testing::RpcHandlerTestServer<HandlerConcept, HandlerType>>
      test_server_;
  MockMapBuilderContext *mock_map_builder_context_;
  std::unique_ptr<MockLocalTrajectoryUploader> mock_local_trajectory_uploader_;
  std::unique_ptr<mapping::testing::MockMapBuilder> mock_map_builder_;
  std::unique_ptr<mapping::testing::MockPoseGraph> mock_pose_graph_;
};

}  // namespace testing
}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_TESTING_HANDLER_TEST_H
