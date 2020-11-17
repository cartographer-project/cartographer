#ifndef CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H
#define CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H

#include <cartographer/sensor/point_cloud.h>
namespace cartographer {
namespace evaluation {

class ScanCloudGenerator {
 public:
  ScanCloudGenerator();
  ScanCloudGenerator(float resolution);
  void generateCube(cartographer::sensor::PointCloud& cloud, float size,
                    float noise_std_dev);
  void generateCubeSlice(cartographer::sensor::PointCloud& cloud, float size,
                    float noise_std_dev, float azimuth, float fov);
  void generateCuboid(cartographer::sensor::PointCloud& cloud, float size_x,
                      float size_y, float size_z, float noise_std_dev);
  void generateCuboidSlice(cartographer::sensor::PointCloud& cloud, float size_x,
                      float size_y, float size_z, float noise_std_dev, float azimuth, float fov);

  enum class ModelType { NONE, CUBE, CUBOID };

 private:
  float resolution_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_SCAN_CLOUD_GENERATOR_H
