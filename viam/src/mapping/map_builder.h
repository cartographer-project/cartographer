#ifndef VIAM_CARTOGRAPHER_MAP_BUILDER_H_
#define VIAM_CARTOGRAPHER_MAP_BUILDER_H_


#include "cartographer/mapping/map_builder.h"
#include "cartographer/common/config.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/viam_read_PCD_file.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"


namespace viam {
namespace mapping {


class MapBuilder {
  public:
  void SetUp(std::string configuration_directory, std::string configuration_basename);

  void BuildMapBuilder();

  cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback();
  
  cartographer::sensor::TimedPointCloudData GetDataFromFile(std::string data_directory, std::string initial_filename, int i);

  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  cartographer::mapping::proto::MapBuilderOptions map_builder_options_;
  cartographer::mapping::proto::TrajectoryBuilderOptions trajectory_builder_options_;
  std::vector<::cartographer::transform::Rigid3d> local_slam_result_poses_;
};

}  // namespace mapping
}  // namespace viam

#endif  // VIAM_CARTOGRAPHER_MAP_BUILDER_H_