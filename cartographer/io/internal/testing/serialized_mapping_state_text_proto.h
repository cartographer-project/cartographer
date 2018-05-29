#ifndef CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_
#define CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_

#include <string>

namespace cartographer {
namespace io {
namespace testing {

static const std::string kSerializationHeaderProtoString = R"(
  format_version: 1
)";

static const std::string kUnsupportedSerializationHeaderProtoString = R"(
  format_version: 123
)";

static const std::string kPoseGraphProtoString = R"(
pose_graph {
  constraint {
    submap_id {
      trajectory_id: 0
      submap_index: 0
    }
    node_id {
      trajectory_id: 0
      node_index: 0
    }
    relative_pose {
      translation {
        x: 0.0
        y: 0.0
        z: 0.0
      }
      rotation {
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
      }
    }
    translation_weight: 1.0
    rotation_weight: 1.0
    tag: INTRA_SUBMAP
  }
  trajectory {
    trajectory_id: 0
    node {
      node_index: 0
      timestamp: 0
      pose {
        translation {
          x: 0.0
          y: 0.0
          z: 0.0
        }
        rotation {
          x: 0.0
          y: 0.0
          z: 0.0
          z: 1.0
        }
      }
    }
    submap {
      submap_index: 0
      pose {
        translation {
          x: 0.0
          y: 0.0
          z: 0.0
        }
        rotation {
          x: 0.0
          y: 0.0
          z: 0.0
          z: 1.0
        }
      }
    }
  }
  landmark_poses {
  }
}
)";

static const std::string kAllTrajectoryBuilderOptionsProtoString = R"(
all_trajectory_builder_options {
  options_with_sensor_ids {
    sensor_id {
      type: RANGE
      id: "laser_scanner_0"
    }
    sensor_id {
      type: IMU
      id: "imu_0"
    }
    trajectory_builder_options {
      trajectory_builder_2d_options {
        min_range: 0.1
        max_range: 40.0
        min_z: 0.0
        max_z: 2.0
      }
      pure_localization: false
      initial_trajectory_pose {
        relative_pose {
          translation {
            x: 0.0
            y: 0.0
            z: 0.0
          }
          rotation {
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
          }
        }
        to_trajectory_id: 0
        timestamp: 0
      }
    }
  }
}
)";

static const std::string kSubmapProtoString = R"(
submap {
  submap_id {
  }
  submap_2d {
  }
}
)";

static const std::string kNodeProtoString = R"(
node {
  node_id {
    trajectory_id: 0
    node_index: 0
  }
  node_data {
    timestamp: 0
    local_pose {
      translation {
        x: 0.0
        y: 0.0
        z: 0.0
      }
      rotation {
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
      }
    }
  }
}
)";

static const std::string kTrajectoryDataProtoString = R"(
trajectory_data {
  trajectory_id: 1
  gravity_constant: 9.81
  imu_calibration {
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  }
}
)";

static const std::string kImuDataProtoString = R"(
imu_data {
  trajectory_id: 5
  imu_data {
  }
}
)";

static const std::string kOdometryDataProtoString = R"(
odometry_data {
  trajectory_id: 654
  odometry_data {
  }
}
)";

static const std::string kFixedFramePoseDataProtoString = R"(
fixed_frame_pose_data {
  trajectory_id: 2
  fixed_frame_pose_data {
  }
}
)";

static const std::string kLandmarkDataProtoString = R"(
landmark_data {
  trajectory_id: 23
  landmark_data {
  }
}
)";

}  // namespace testing
}  // namespace io
}  // namespace cartographer

#endif  // ifndef CARTOGRAPHER_IO_INTERNAL_TESTING_SERIALIZED_TEST_TEXT_PROTO_H_
