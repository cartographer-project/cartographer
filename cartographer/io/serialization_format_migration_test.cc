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

#include "cartographer/io/serialization_format_migration.h"

#include <functional>
#include <memory>

#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace {

class SubmapHistogramMigrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    CreateSubmap();
    CreateNode();
    CreatePoseGraphWithNodeToSubmapConstraint();
  }

 private:
  void CreateSubmap() {
    submap_ = mapping::testing::CreateFakeSubmap3D();
    submap_.mutable_submap_3d()->clear_rotational_scan_matcher_histogram();
  }

  void CreateNode() {
    node_ = mapping::testing::CreateFakeNode();
    constexpr int histogram_size = 10;
    for (int i = 0; i < histogram_size; ++i) {
      node_.mutable_node_data()->add_rotational_scan_matcher_histogram(1.f);
    }
  }

  void CreatePoseGraphWithNodeToSubmapConstraint() {
    mapping::proto::PoseGraph::Constraint* constraint =
        pose_graph_.add_constraint();
    constraint->set_tag(mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP);
    *constraint->mutable_node_id() = node_.node_id();
    *constraint->mutable_submap_id() = submap_.submap_id();
    *constraint->mutable_relative_pose() =
        transform::ToProto(transform::Rigid3d::Identity());
  }

 protected:
  mapping::proto::PoseGraph pose_graph_;
  mapping::proto::Submap submap_;
  mapping::proto::Node node_;
};

TEST_F(SubmapHistogramMigrationTest,
       SubmapHistogramGenerationFromTrajectoryNodes) {
  mapping::MapById<mapping::SubmapId, mapping::proto::Submap>
      submap_id_to_submap;
  mapping::SubmapId submap_id(submap_.submap_id().trajectory_id(),
                              submap_.submap_id().submap_index());
  submap_id_to_submap.Insert(submap_id, submap_);

  mapping::MapById<mapping::NodeId, mapping::proto::Node> node_id_to_node;
  mapping::NodeId node_id(node_.node_id().trajectory_id(),
                          node_.node_id().node_index());
  node_id_to_node.Insert(node_id, node_);

  const auto submap_id_to_submap_migrated =
      MigrateSubmapFormatVersion1ToVersion2(submap_id_to_submap,
                                            node_id_to_node, pose_graph_);

  EXPECT_EQ(submap_id_to_submap_migrated.size(), submap_id_to_submap.size());
  const mapping::proto::Submap& migrated_submap =
      submap_id_to_submap_migrated.at(submap_id);

  EXPECT_FALSE(migrated_submap.has_submap_2d());
  EXPECT_TRUE(migrated_submap.has_submap_3d());
  const mapping::proto::Submap3D& migrated_submap_3d =
      migrated_submap.submap_3d();
  const mapping::proto::TrajectoryNodeData& node_data = node_.node_data();
  EXPECT_EQ(migrated_submap_3d.rotational_scan_matcher_histogram_size(),
            node_data.rotational_scan_matcher_histogram_size());
  for (int i = 0; i < node_data.rotational_scan_matcher_histogram_size(); ++i) {
    EXPECT_EQ(migrated_submap_3d.rotational_scan_matcher_histogram(i),
              node_data.rotational_scan_matcher_histogram(i));
  }
}

}  // namespace
}  // namespace io
}  // namespace cartographer
