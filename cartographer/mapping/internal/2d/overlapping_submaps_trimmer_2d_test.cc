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

#include "cartographer/mapping/internal/2d/overlapping_submaps_trimmer_2d.h"

#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/testing/fake_trimmable.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace {

using ::cartographer::transform::Rigid2d;
using ::cartographer::transform::Rigid3d;
using ::testing::ElementsAre;
using ::testing::Field;
using ::testing::IsEmpty;

class OverlappingSubmapsTrimmer2DTest : public ::testing::Test {
 protected:
  // Creates a submap with num_cells x num_cells grid.
  void AddSquareSubmap(const Rigid2d& global_to_submap_frame,
                       const Rigid2d& local_to_submap_frame,
                       const Eigen::Vector2d& submap_corner, int submap_index,
                       int num_cells, bool is_finished) {
    proto::Submap2D submap_2d;
    submap_2d.set_num_range_data(1);
    submap_2d.set_finished(is_finished);
    *submap_2d.mutable_local_pose() =
        transform::ToProto(transform::Embed3D(local_to_submap_frame));

    auto* grid = submap_2d.mutable_grid();
    for (int i = 0; i < num_cells * num_cells; ++i) {
      grid->add_cells(1);
    }

    auto* map_limits = grid->mutable_limits();
    map_limits->set_resolution(1.0);
    *map_limits->mutable_max() = transform::ToProto(submap_corner);
    map_limits->mutable_cell_limits()->set_num_x_cells(num_cells);
    map_limits->mutable_cell_limits()->set_num_y_cells(num_cells);

    auto* know_cells_box = grid->mutable_known_cells_box();
    know_cells_box->set_min_x(0);
    know_cells_box->set_min_y(0);
    know_cells_box->set_max_x(num_cells - 1);
    know_cells_box->set_max_y(num_cells - 1);

    grid->mutable_probability_grid_2d();
    fake_pose_graph_.mutable_submap_data()->Insert(
        {0 /* trajectory_id */, submap_index},
        {std::make_shared<const Submap2D>(submap_2d, &conversion_tables_),
         transform::Embed3D(global_to_submap_frame)});
  }

  void AddTrajectoryNode(int node_index, int64 timestamp) {
    TrajectoryNode::Data data;
    data.time = common::FromUniversal(timestamp);

    fake_pose_graph_.mutable_trajectory_nodes()->Insert(
        NodeId{0 /* trajectory_id */, node_index},
        {std::make_shared<const TrajectoryNode::Data>(data), {} /* pose */});
  }

  void AddConstraint(int submap_index, int node_index, bool is_intra_submap) {
    fake_pose_graph_.mutable_constraints()->push_back(
        {SubmapId{0 /* trajectory_id */, submap_index},
         NodeId{0 /* trajectory_id */, node_index},
         {} /* pose */,
         is_intra_submap ? PoseGraphInterface::Constraint::INTRA_SUBMAP
                         : PoseGraphInterface::Constraint::INTER_SUBMAP});
  }

  ValueConversionTables conversion_tables_;
  testing::FakeTrimmable fake_pose_graph_;
};

::testing::Matcher<const SubmapId&> EqualsSubmapId(const SubmapId& expected) {
  return ::testing::AllOf(
      Field(&SubmapId::trajectory_id, expected.trajectory_id),
      Field(&SubmapId::submap_index, expected.submap_index));
}

TEST_F(OverlappingSubmapsTrimmer2DTest, EmptyPoseGraph) {
  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      0 /* min_covered_area */,
                                      0 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(), IsEmpty());
}

TEST_F(OverlappingSubmapsTrimmer2DTest, TrimOneOfTwoOverlappingSubmaps) {
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  0 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  1 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(0 /* node_index */, 1000 /* timestamp */);
  AddTrajectoryNode(1 /* node_index */, 2000 /* timestamp */);
  AddConstraint(0 /*submap_index*/, 0 /*node_index*/, true);
  AddConstraint(1 /*submap_index*/, 1 /*node_index*/, true);

  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      0 /* min_covered_area */,
                                      0 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(),
              ElementsAre(EqualsSubmapId({0, 0})));
}

TEST_F(OverlappingSubmapsTrimmer2DTest, TestMinAddedSubmapsCountParam) {
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  0 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  1 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(0 /* node_index */, 1000 /* timestamp */);
  AddTrajectoryNode(1 /* node_index */, 2000 /* timestamp */);
  AddConstraint(0 /*submap_index*/, 0 /*node_index*/, true);
  AddConstraint(1 /*submap_index*/, 1 /*node_index*/, true);

  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      0 /* min_covered_area */,
                                      2 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(), IsEmpty());

  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  2 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(2 /* node_index */, 3000 /* timestamp */);
  AddConstraint(2 /*submap_index*/, 2 /*node_index*/, true);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(),
              ElementsAre(EqualsSubmapId({0, 0}), EqualsSubmapId({0, 1})));
}

TEST_F(OverlappingSubmapsTrimmer2DTest, DoNotTrimUnfinishedSubmap) {
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  0 /* submap_index */, 1 /* num_cells */,
                  false /* is_finished */);
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  1 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(0 /* node_index */, 1000 /* timestamp */);
  AddTrajectoryNode(1 /* node_index */, 2000 /* timestamp */);
  AddConstraint(0 /*submap_index*/, 0 /*node_index*/, true);
  AddConstraint(1 /*submap_index*/, 1 /*node_index*/, true);

  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      0 /* min_covered_area */,
                                      0 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(), IsEmpty());
}

TEST_F(OverlappingSubmapsTrimmer2DTest, UseOnlyIntraSubmapsToComputeFreshness) {
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  0 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  1 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(0 /* node_index */, 1000 /* timestamp */);
  AddTrajectoryNode(1 /* node_index */, 2000 /* timestamp */);
  AddTrajectoryNode(2 /* node_index */, 3000 /* timestamp */);
  AddConstraint(0 /*submap_index*/, 0 /*node_index*/, false);
  AddConstraint(0 /*submap_index*/, 2 /*node_index*/, true);
  AddConstraint(1 /*submap_index*/, 1 /*node_index*/, true);

  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      0 /* min_covered_area */,
                                      0 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(),
              ElementsAre(EqualsSubmapId({0, 1})));
}

// This test covers two 4-cells submaps that overlap each other displaced like:
//    ___
//  _|   |
// | |_ _|
// |___|
//
// The background submap should be trimmed, since it has only 3 cells
// not-covered by another submap.
TEST_F(OverlappingSubmapsTrimmer2DTest, TrimSubmapWithLowCoveredCellsCount) {
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  0 /* submap_index */, 2 /* num_cells */,
                  true /* is_finished */);
  AddSquareSubmap(Rigid2d::Translation(
                      Eigen::Vector2d(1., 1.)) /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  1 /* submap_index */, 2 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(0 /* node_index */, 1000 /* timestamp */);
  AddTrajectoryNode(1 /* node_index */, 2000 /* timestamp */);
  AddConstraint(0 /*submap_index*/, 0 /*node_index*/, true);
  AddConstraint(1 /*submap_index*/, 1 /*node_index*/, true);

  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      4 /* min_covered_cells_count */,
                                      0 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(),
              ElementsAre(EqualsSubmapId({0, 0})));
}

TEST_F(OverlappingSubmapsTrimmer2DTest, TestTransformations) {
  AddSquareSubmap(Rigid2d::Identity() /* global_from_submap_frame */,
                  Rigid2d::Identity() /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  0 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  const Rigid2d transform(Eigen::Vector2d(1., 1.), M_PI / 2);
  AddSquareSubmap(transform /* global_from_submap_frame */,
                  transform /* local_from_submap_frame */,
                  Eigen::Vector2d(1., 1.) /* submap corner */,
                  1 /* submap_index */, 1 /* num_cells */,
                  true /* is_finished */);
  AddTrajectoryNode(0 /* node_index */, 1000 /* timestamp */);
  AddTrajectoryNode(1 /* node_index */, 2000 /* timestamp */);
  AddConstraint(0 /*submap_index*/, 0 /*node_index*/, true);
  AddConstraint(1 /*submap_index*/, 1 /*node_index*/, true);

  OverlappingSubmapsTrimmer2D trimmer(1 /* fresh_submaps_count */,
                                      0 /* min_covered_area */,
                                      0 /* min_added_submaps_count */);
  trimmer.Trim(&fake_pose_graph_);
  EXPECT_THAT(fake_pose_graph_.trimmed_submaps(),
              ElementsAre(EqualsSubmapId({0, 0})));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
