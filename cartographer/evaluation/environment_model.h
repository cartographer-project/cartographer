#ifndef CARTOGRAPHER_EVALUATION_ENVIRONMENT_MODEL_H
#define CARTOGRAPHER_EVALUATION_ENVIRONMENT_MODEL_H

#include <cartographer/sensor/point_cloud.h>
namespace cartographer {
namespace evaluation {

class EnvironmentModel {
 public:
  EnvironmentModel();
  bool checkRayIntersection(Eigen::Vector3f origin, Eigen::Vector3f direction, float* intersection_distance);

 private:
  std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> planes_;  // origin, normal
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_ENVIRONMENT_MODEL_H
