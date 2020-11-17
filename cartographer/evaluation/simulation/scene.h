#ifndef CARTOGRAPHER_EVALUATION_SCENE_H
#define CARTOGRAPHER_EVALUATION_SCENE_H

#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace evaluation {

class Scene {
 public:
  Scene();
  void AddSphere(const Eigen::Vector3f& origin, float radius);
  void AddBox(const Eigen::Vector3f& center, const Eigen::Vector3f& size);
  Eigen::Vector3f CollideWithScene(const Eigen::Vector3f& from,
                                   const Eigen::Vector3f& to,
                                   float& ratio) const;

 private:
  Eigen::Vector3f CollideWithSpheres(const Eigen::Vector3f& from,
                                     const Eigen::Vector3f& to,
                                     float& ratio) const;
  Eigen::Vector3f CollideWithBoxes(const Eigen::Vector3f& from,
                                   const Eigen::Vector3f& to,
                                   float& ratio) const;
  std::vector<std::pair<Eigen::Vector3f, float>> spheres_;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> boxes_;
};

}  // namespace evaluation
}  // namespace cartographer
#endif  // CARTOGRAPHER_EVALUATION_SCENE_H
