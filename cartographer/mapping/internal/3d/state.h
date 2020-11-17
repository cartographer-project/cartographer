
#ifndef CARTOGRAPHER_MAPPING_3D_STATE_H_
#define CARTOGRAPHER_MAPPING_3D_STATE_H_

#include <array>
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct State {
  std::array<double, 3> translation;
  std::array<double, 4> rotation;  // Rotation quaternion as (w, x, y, z).
  std::array<double, 3> velocity;

  State(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation,
        const Eigen::Vector3d& velocity)
      : translation{{translation.x(), translation.y(), translation.z()}},
        rotation{{rotation.w(), rotation.x(), rotation.y(), rotation.z()}},
        velocity{{velocity.x(), velocity.y(), velocity.z()}} {}

  Eigen::Quaterniond ToQuaternion() const {
    return Eigen::Quaterniond(rotation[0], rotation[1], rotation[2],
                              rotation[3]);
  }

  transform::Rigid3d ToRigid() const {
    return transform::Rigid3d(
        Eigen::Vector3d(translation[0], translation[1], translation[2]),
        ToQuaternion());
  }
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_STATE_H_
