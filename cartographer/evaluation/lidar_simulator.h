#ifndef CARTOGRAPHER_EVALUATION_LIDAR_SIMULATOR_H
#define CARTOGRAPHER_EVALUATION_LIDAR_SIMULATOR_H

#include <cartographer/sensor/point_cloud.h>
#include <cartographer/evaluation/environment_model.h>
namespace cartographer {
namespace evaluation {

class LidarSimulator {
 public:
  LidarSimulator();

  void generateScan(const EnvironmentModel& environment);

 private:
  float max_range_;
  float depth_stdv_;
  float resolution_vertical_;
  float resolution_horizontal_;
  float vertical_range_;
  float horizontal_range_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_LIDAR_SIMULATOR_H
