#include "cartographer/evaluation/scan_cloud_generator.h"

#include <random>

namespace cartographer {
namespace evaluation {
namespace {
static std::default_random_engine e1(42);
}  // namespace

ScanCloudGenerator::ScanCloudGenerator() : resolution_(0.02) {}

ScanCloudGenerator::ScanCloudGenerator(float resolution)
    : resolution_(resolution) {}

void ScanCloudGenerator::generateCube(cartographer::sensor::PointCloud& cloud,
                                      float size, float noise_std_dev) {
  generateCuboid(cloud, size, size, size, noise_std_dev);
}


void ScanCloudGenerator::generateCubeSlice(cartographer::sensor::PointCloud& cloud, float size,
                       float noise_std_dev, float azimuth, float fov) {
  cartographer::sensor::PointCloud unfiltered_cloud;
  generateCube(unfiltered_cloud, size, noise_std_dev);
  cloud = cartographer::sensor::PointCloud();
  for(const auto& point : unfiltered_cloud) {
    float delta = std::abs(std::atan2(point.position.y(), point.position.x()) - azimuth);
    if(delta > M_PI) {
      delta = std::abs(delta - 2.f * M_PI);
    }
    if(delta < fov || delta > M_PI - fov) {
      cloud.push_back(point);
    }
  }
}


void ScanCloudGenerator::generateCuboid(cartographer::sensor::PointCloud& cloud,
                                        float size_x, float size_y,
                                        float size_z, float noise_std_dev) {
  std::random_device r;

  std::normal_distribution<float> normal_distribution(0,
                                                      noise_std_dev);  // 0.01

  cloud.clear();
  float x_min = -size_x / 2.0f;
  float x_max = size_x / 2.0;
  float y_min = -size_y / 2.0;
  float y_max = size_y / 2.0;
  float z_min = -size_z / 2.0;
  float z_max = size_z / 2.0;

  std::uniform_real_distribution<double> error_translation_direction(-M_PI,
                                                                     M_PI);
  double e_orientation;
  double e_scale = noise_std_dev == 0.0 ? 0.0 : noise_std_dev;
  double e_x;
  double e_y;

  for (float z = z_min + resolution_; z < z_max - resolution_ + 1e-5;
       z += resolution_) {
    for (float x = x_min + resolution_; x < x_max - resolution_ + 1e-5;
         x += resolution_) {
      float y = y_min;
      e_orientation = normal_distribution(e1);
      e_x = x / sqrt(x * x + y * y) * e_orientation;
      e_y = y / sqrt(x * x + y * y) * e_orientation;
      cloud.push_back({Eigen::Vector3f(x + e_x, y + e_y, z)});
//      y = y_max;
//      e_orientation = normal_distribution(e1);
//      e_x = x / sqrt(x * x + y * y) * e_orientation;
//      e_y = y / sqrt(x * x + y * y) * e_orientation;
//      cloud.push_back({Eigen::Vector3f(x + e_x, y + e_y, z)});
    }
    for (float y = y_min; y < y_max + 1e-5; y += resolution_) {
      float x = x_min;
      e_orientation = normal_distribution(e1);
      e_x = x / sqrt(x * x + y * y) * e_orientation;
      e_y = y / sqrt(x * x + y * y) * e_orientation;
      cloud.push_back({Eigen::Vector3f(x + e_x, y + e_y, z)});
//      x = x_max;
//      e_orientation = normal_distribution(e1);
//      e_x = x / sqrt(x * x + y * y) * e_orientation;
//      e_y = y / sqrt(x * x + y * y) * e_orientation;
//      cloud.push_back({Eigen::Vector3f(x + e_x, y + e_y, z)});
    }
  }

  for (float x = x_min + resolution_; x < x_max - resolution_ + 1e-5;
       x += resolution_) {
    for (float y = y_min; y < y_max + 1e-5; y += resolution_) {
      cloud.push_back({Eigen::Vector3f(x, y, z_min)});
//      cloud.push_back({Eigen::Vector3f(x, y, z_max)});
    }
  }
}


void ScanCloudGenerator::generateCuboidSlice(cartographer::sensor::PointCloud& cloud, float size_x,
                         float size_y, float size_z, float noise_std_dev, float azimuth, float fov) {
  cartographer::sensor::PointCloud unfiltered_cloud;
  generateCuboid(unfiltered_cloud, size_x, size_y, size_z, noise_std_dev);
  cloud = cartographer::sensor::PointCloud();
  for(const auto& point : unfiltered_cloud) {
    float delta = std::abs(std::atan2(point.position.y(), point.position.x()) - azimuth);
    if(delta > M_PI) {
      delta = std::abs(delta - 2.f * M_PI);
    }
    if(delta < fov || delta > M_PI + fov) {
      cloud.push_back(point);
    }
  }

}

}  // namespace evaluation
}  // namespace cartographer
