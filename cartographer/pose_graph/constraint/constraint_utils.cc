#include "cartographer/pose_graph/constraint/constraint_utils.h"

namespace cartographer {
namespace pose_graph {

void AddPose2D(Pose2D* pose, ceres::Problem* problem) {
  auto pose_2d = pose->mutable_pose_2d();
  problem->AddParameterBlock(pose_2d->data(), pose_2d->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(pose_2d->data());
  }
}

void AddPose3D(Pose3D* pose, ceres::Problem* problem) {
  auto transation = pose->mutable_translation();
  auto rotation = pose->mutable_rotation();
  problem->AddParameterBlock(transation->data(), transation->size());
  problem->AddParameterBlock(rotation->data(), rotation->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(transation->data());
    problem->SetParameterBlockConstant(rotation->data());
  }
}

void AddImuCalibration(ImuCalibration* pose, ceres::Problem* problem) {
  auto gravity = pose->mutable_gravity_constant();
  auto orientation = pose->mutable_orientation();
  problem->AddParameterBlock(gravity, 1);
  problem->AddParameterBlock(orientation->data(), orientation->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(gravity);
    problem->SetParameterBlockConstant(orientation->data());
  }
}

}  // namespace pose_graph
}  // namespace cartographer
