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

#ifndef CARTOGRAPHER_GRPC_TESTING_HANDLER_TEST_H
#define CARTOGRAPHER_GRPC_TESTING_HANDLER_TEST_H

#include "cartographer/common/make_unique.h"
#include "cartographer_grpc/framework/testing/rpc_handler_test_server.h"
#include "cartographer_grpc/testing/mock_local_trajectory_uploader.h"
#include "cartographer_grpc/testing/mock_map_builder.h"
#include "cartographer_grpc/testing/mock_map_builder_context.h"
#include "cartographer_grpc/testing/mock_pose_graph.h"
#include "gtest/gtest.h"

namespace cartographer_grpc {
namespace testing {

using ::testing::Return;
using ::testing::Test;

template <typename HandlerType>
class HandlerTest : public Test {
 public:
  void SetUp() override {
    test_server_ = cartographer::common::make_unique<
        framework::testing::RpcHandlerTestServer<HandlerType>>(
        cartographer::common::make_unique<MockMapBuilderContext>());
    mock_map_builder_context_ =
        test_server_
            ->template GetUnsynchronizedContext<MockMapBuilderContext>();
    mock_local_trajectory_uploader_ =
        cartographer::common::make_unique<MockLocalTrajectoryUploader>();
    mock_map_builder_ = cartographer::common::make_unique<MockMapBuilder>();
    mock_pose_graph_ = cartographer::common::make_unique<MockPoseGraph>();

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
  std::unique_ptr<framework::testing::RpcHandlerTestServer<HandlerType>>
      test_server_;
  MockMapBuilderContext* mock_map_builder_context_;
  std::unique_ptr<MockLocalTrajectoryUploader> mock_local_trajectory_uploader_;
  std::unique_ptr<MockMapBuilder> mock_map_builder_;
  std::unique_ptr<MockPoseGraph> mock_pose_graph_;
};

}  // namespace testing
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_TESTING_HANDLER_TEST_H
