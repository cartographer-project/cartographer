/*
 * Copyright 2016 The Cartographer Authors
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

#include "drawable_submap.h"

#include <OgreGpuProgramParams.h>
#include <OgreImage.h>
#include <cartographer/common/port.h>
#include <eigen_conversions/eigen_msg.h>
#include <google_cartographer_msgs/SubmapQuery.h>
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <future>
#include <sstream>
#include <string>

namespace cartographer_ros {
namespace rviz {

namespace {

constexpr std::chrono::milliseconds kMinQueryDelayInMs(250);
constexpr char kMapFrame[] = "/map";
constexpr char kSubmapTexturePrefix[] = "SubmapTexture";
constexpr char kManualObjectPrefix[] = "ManualObjectSubmap";
constexpr char kSubmapMaterialPrefix[] = "SubmapMaterial";
constexpr char kSubmapSourceMaterialName[] = "google_cartographer/Submap";

// Distance before which the submap will be shown at full opacity, and distance
// over which the submap will then fade out.
constexpr double kFadeOutStartDistanceInMeters = 1.;
constexpr double kFadeOutDistanceInMeters = 2.;
constexpr float kAlphaUpdateThreshold = 0.2f;

std::string GetSubmapIdentifier(const int trajectory_id, const int submap_id) {
  return std::to_string(trajectory_id) + "-" + std::to_string(submap_id);
}

}  // namespace

DrawableSubmap::DrawableSubmap(const int submap_id, const int trajectory_id,
                               ::rviz::FrameManager* const frame_manager,
                               Ogre::SceneManager* const scene_manager)
    : frame_manager_(frame_manager),
      scene_node_(scene_manager->getRootSceneNode()->createChildSceneNode()),
      manual_object_(scene_manager->createManualObject(
          kManualObjectPrefix + GetSubmapIdentifier(trajectory_id, submap_id))),
      submap_id_(submap_id),
      trajectory_id_(trajectory_id),
      last_query_timestamp_(0),
      query_in_progress_(false),
      texture_count_(0),
      current_alpha_(0) {
  material_ = Ogre::MaterialManager::getSingleton().getByName(
      kSubmapSourceMaterialName);
  material_ = material_->clone(kSubmapMaterialPrefix +
                               GetSubmapIdentifier(trajectory_id_, submap_id));
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthWriteEnabled(false);
  scene_node_->attachObject(manual_object_);
  connect(this, SIGNAL(RequestSucceeded()), this, SLOT(OnRequestSuccess()));
}

DrawableSubmap::~DrawableSubmap() {
  Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.setNull();
  }
}

bool DrawableSubmap::Update(
    const ::google_cartographer_msgs::SubmapEntry& metadata,
    ros::ServiceClient* const client) {
  ::cartographer::common::MutexLocker locker(&mutex_);
  tf::poseMsgToEigen(metadata.pose, submap_pose_);
  const bool newer_version_available = version_ < metadata.submap_version;
  const std::chrono::milliseconds now =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  const bool recently_queried =
      last_query_timestamp_ + kMinQueryDelayInMs > now;
  if (!newer_version_available || recently_queried || query_in_progress_) {
    return false;
  }
  query_in_progress_ = true;
  last_query_timestamp_ = now;
  last_query_slice_height_ = metadata.pose.position.z;
  QuerySubmap(submap_id_, trajectory_id_, client);
  return true;
}

void DrawableSubmap::QuerySubmap(const int submap_id, const int trajectory_id,
                                 ros::ServiceClient* const client) {
  rpc_request_future_ = std::async(
      std::launch::async, [this, submap_id, trajectory_id, client]() {
        ::google_cartographer_msgs::SubmapQuery srv;
        srv.request.submap_id = submap_id;
        srv.request.trajectory_id = trajectory_id;
        if (client->call(srv)) {
          response_.reset(new ::google_cartographer_msgs::SubmapQuery::Response(
              srv.response));
          Q_EMIT RequestSucceeded();
        } else {
          OnRequestFailure();
        }
      });
}

void DrawableSubmap::OnRequestSuccess() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  version_ = response_->submap_version;
  resolution_ = response_->resolution;
  width_ = response_->width;
  height_ = response_->height;
  slice_height_ = last_query_slice_height_;
  std::string compressed_cells(response_->cells.begin(),
                               response_->cells.end());
  cells_.clear();
  ::cartographer::common::FastGunzipString(compressed_cells, &cells_);
  Eigen::Affine3d slice_pose;
  tf::poseMsgToEigen(response_->slice_pose, slice_pose);
  tf::poseEigenToMsg(submap_pose_ * slice_pose, transformed_pose_);
  response_.reset();
  query_in_progress_ = false;
  UpdateSceneNode();
}

void DrawableSubmap::OnRequestFailure() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  query_in_progress_ = false;
}

bool DrawableSubmap::QueryInProgress() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  return query_in_progress_;
}

void DrawableSubmap::UpdateSceneNode() {
  // The call to Ogre's loadRawData below does not work with an RG texture,
  // therefore we create an RGB one whose blue channel is always 0.
  std::vector<char> rgb;
  for (int i = 0; i < height_; ++i) {
    for (int j = 0; j < width_; ++j) {
      auto r = cells_[(i * width_ + j) * 2];
      auto g = cells_[(i * width_ + j) * 2 + 1];
      rgb.push_back(r);
      rgb.push_back(g);
      rgb.push_back(0.);
    }
  }

  manual_object_->clear();
  const float metric_width = resolution_ * width_;
  const float metric_height = resolution_ * height_;

  manual_object_->begin(material_->getName(),
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    {
      // Bottom left
      manual_object_->position(-metric_height, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(-metric_height, -metric_width, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(-metric_height, -metric_width, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(0.0f, -metric_width, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }
  }

  manual_object_->end();

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(rgb.data(), rgb.size()));

  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
    texture_.setNull();
  }
  const std::string texture_name =
      kSubmapTexturePrefix + GetSubmapIdentifier(trajectory_id_, submap_id_) +
      std::to_string(texture_count_);
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
      texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, width_, height_, Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D, 0);
  ++texture_count_;

  Ogre::Pass* const pass = material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBF_ONE, Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
  Ogre::TextureUnitState* const texture_unit =
      pass->getNumTextureUnitStates() > 0 ? pass->getTextureUnitState(0)
                                          : pass->createTextureUnitState();

  texture_unit->setTextureName(texture_->getName());
  texture_unit->setTextureFiltering(Ogre::TFO_NONE);
}

void DrawableSubmap::Transform(const ros::Time& ros_time) {
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  frame_manager_->transform(kMapFrame, ros_time, transformed_pose_, position,
                            orientation);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void DrawableSubmap::SetAlpha(const double current_tracking_z) {
  const double distance_z = std::abs(slice_height_ - current_tracking_z);
  const double fade_distance =
      std::max(distance_z - kFadeOutStartDistanceInMeters, 0.);
  const float alpha =
      (float)std::max(0., 1. - fade_distance / kFadeOutDistanceInMeters);

  const Ogre::GpuProgramParametersSharedPtr parameters =
      material_->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
  parameters->setNamedConstant("u_alpha", UpdateAlpha(alpha));
}

float DrawableSubmap::UpdateAlpha(const float target_alpha) {
  if (std::abs(target_alpha - current_alpha_) > kAlphaUpdateThreshold ||
      target_alpha == 0.f || target_alpha == 1.f) {
    current_alpha_ = target_alpha;
  }
  return current_alpha_;
}

}  // namespace rviz
}  // namespace cartographer_ros
