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

#include "submaps_display.h"

#include <OgreColourValue.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreOverlay.h>
#include <OgreOverlayContainer.h>
#include <OgreOverlayManager.h>
#include <OgreRenderTexture.h>
#include <OgreResourceGroupManager.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <cartographer/common/mutex.h>
#include <geometry_msgs/TransformStamped.h>
#include <google_cartographer_msgs/SubmapList.h>
#include <google_cartographer_msgs/SubmapQuery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>

namespace cartographer_ros {
namespace rviz {

namespace {

constexpr int kMaxOnGoingRequests = 6;
constexpr char kMaterialsDirectory[] = "/ogre_media/materials";
constexpr char kGlsl120Directory[] = "/glsl120";
constexpr char kScriptsDirectory[] = "/scripts";
constexpr char kScreenBlitMaterialName[] = "ScreenBlitMaterial";
constexpr char kScreenBlitSourceMaterialName[] =
    "google_cartographer/ScreenBlit";
constexpr char kSubmapsRttPrefix[] = "SubmapsRtt";
constexpr char kMapTextureName[] = "MapTexture";
constexpr char kMapOverlayName[] = "MapOverlay";
constexpr char kSubmapsSceneCameraName[] = "SubmapsSceneCamera";
constexpr char kSubmapTexturesGroup[] = "SubmapTexturesGroup";
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultTrackingFrame[] = "base_link";

}  // namespace

SubmapsDisplay::SubmapsDisplay()
    : Display(),
      rtt_count_(0),
      scene_manager_listener_([this]() { UpdateMapTexture(); }),
      tf_listener_(tf_buffer_) {
  connect(this, SIGNAL(SubmapListUpdated()), this, SLOT(RequestNewSubmaps()));
  topic_property_ = new ::rviz::RosTopicProperty(
      "Topic", "",
      QString::fromStdString(ros::message_traits::datatype<
                             ::google_cartographer_msgs::SubmapList>()),
      "google_cartographer_msgs::SubmapList topic to subscribe to.", this,
      SLOT(UpdateTopic()));
  submap_query_service_property_ = new ::rviz::StringProperty(
      "Submap query service", "", "Submap query service to connect to.", this,
      SLOT(UpdateSubmapQueryServiceName()));
  map_frame_property_ = new ::rviz::StringProperty(
      "Map frame", kDefaultMapFrame, "Map frame, used for fading out submaps.",
      this);
  tracking_frame_property_ = new ::rviz::StringProperty(
      "Tracking frame", kDefaultTrackingFrame,
      "Tracking frame, used for fading out submaps.", this);
  client_ =
      update_nh_.serviceClient<::google_cartographer_msgs::SubmapQuery>("");
  const std::string package_path = ::ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory, "FileSystem", ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kGlsl120Directory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      package_path + kMaterialsDirectory + kScriptsDirectory, "FileSystem",
      ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

SubmapsDisplay::~SubmapsDisplay() {
  Unsubscribe();
  client_.shutdown();
  Clear();
}

void SubmapsDisplay::onInitialize() {
  submaps_scene_manager_ =
      Ogre::Root::getSingletonPtr()->createSceneManager(Ogre::ST_GENERIC);
  submaps_scene_camera_ =
      submaps_scene_manager_->createCamera(kSubmapsSceneCameraName);
  submap_scene_material_ = Ogre::MaterialManager::getSingleton().create(
      kMapTextureName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  screen_blit_material_ = Ogre::MaterialManager::getSingleton().getByName(
      kScreenBlitSourceMaterialName);
  screen_blit_material_ = screen_blit_material_->clone(kScreenBlitMaterialName);
  screen_blit_material_->setReceiveShadows(false);
  screen_blit_material_->getTechnique(0)->setLightingEnabled(false);
  screen_blit_material_->setDepthWriteEnabled(false);

  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
  overlay_ = overlay_manager.create(kMapOverlayName);
  panel_ = static_cast<Ogre::OverlayContainer*>(
      overlay_manager.createOverlayElement("Panel", "PanelName"));
  overlay_->add2D(panel_);
  panel_->setPosition(0.0, 0.0);
  panel_->setDimensions(1., 1.);
  panel_->setMaterialName(kScreenBlitMaterialName);

  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
      kSubmapTexturesGroup);

  scene_manager_->addListener(&scene_manager_listener_);
  UpdateTopic();
}

void SubmapsDisplay::UpdateTopic() {
  Unsubscribe();
  Clear();
  Subscribe();
}

void SubmapsDisplay::UpdateSubmapQueryServiceName() {
  Unsubscribe();
  Clear();
  client_.shutdown();
  client_ = update_nh_.serviceClient<::google_cartographer_msgs::SubmapQuery>(
      submap_query_service_property_->getStdString());
  Subscribe();
}

void SubmapsDisplay::reset() {
  Display::reset();

  Clear();
  UpdateTopic();
}

void SubmapsDisplay::onEnable() { Subscribe(); }

void SubmapsDisplay::onDisable() {
  Unsubscribe();
  Clear();
}

void SubmapsDisplay::Subscribe() {
  if (!isEnabled()) {
    return;
  }

  if (!topic_property_->getTopic().isEmpty()) {
    try {
      submap_list_subscriber_ =
          update_nh_.subscribe(topic_property_->getTopicStd(), 1,
                               &SubmapsDisplay::IncomingSubmapList, this,
                               ros::TransportHints().reliable());
      setStatus(::rviz::StatusProperty::Ok, "Topic", "OK");
    } catch (ros::Exception& e) {
      setStatus(::rviz::StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void SubmapsDisplay::Unsubscribe() { submap_list_subscriber_.shutdown(); }

void SubmapsDisplay::IncomingSubmapList(
    const ::google_cartographer_msgs::SubmapList::ConstPtr& msg) {
  submap_list_ = *msg;
  Q_EMIT SubmapListUpdated();
}

void SubmapsDisplay::Clear() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  submaps_scene_manager_->clearScene();
  if (!rttTexture_.isNull()) {
    rttTexture_->unload();
    rttTexture_.setNull();
  }
  Ogre::ResourceGroupManager::getSingleton().clearResourceGroup(
      kSubmapTexturesGroup);
  trajectories_.clear();
  overlay_->hide();
}

void SubmapsDisplay::RequestNewSubmaps() {
  ::cartographer::common::MutexLocker locker(&mutex_);
  for (int trajectory_id = 0; trajectory_id < submap_list_.trajectory.size();
       ++trajectory_id) {
    if (trajectory_id >= trajectories_.size()) {
      trajectories_.emplace_back(new Trajectory);
    }
    const std::vector<::google_cartographer_msgs::SubmapEntry>& submap_entries =
        submap_list_.trajectory[trajectory_id].submap;
    if (submap_entries.empty()) {
      return;
    }
    for (int submap_id = trajectories_[trajectory_id]->Size();
         submap_id < submap_entries.size(); ++submap_id) {
      trajectories_[trajectory_id]->Add(submap_id, trajectory_id,
                                        context_->getFrameManager(),
                                        submaps_scene_manager_);
    }
  }
  int num_ongoing_requests = 0;
  for (const auto& trajectory : trajectories_) {
    for (int i = 0; i < trajectory->Size(); ++i) {
      if (trajectory->Get(i).QueryInProgress()) {
        ++num_ongoing_requests;
        if (num_ongoing_requests == kMaxOnGoingRequests) {
          return;
        }
      }
    }
  }
  for (int trajectory_id = 0; trajectory_id < submap_list_.trajectory.size();
       ++trajectory_id) {
    const std::vector<::google_cartographer_msgs::SubmapEntry>& submap_entries =
        submap_list_.trajectory[trajectory_id].submap;
    for (int submap_id = submap_entries.size() - 1; submap_id >= 0;
         --submap_id) {
      if (trajectories_[trajectory_id]->Get(submap_id).Update(
              submap_entries[submap_id], &client_)) {
        ++num_ongoing_requests;
        if (num_ongoing_requests == kMaxOnGoingRequests) {
          return;
        }
      }
    }
  }
}

void SubmapsDisplay::UpdateMapTexture() {
  if (trajectories_.empty()) {
    return;
  }
  const int width = scene_manager_->getCurrentViewport()->getActualWidth();
  const int height = scene_manager_->getCurrentViewport()->getActualHeight();
  if (!rttTexture_.isNull()) {
    rttTexture_->unload();
    rttTexture_.setNull();
  }
  // If the rtt texture is freed every time UpdateMapTexture() is called, the
  // code slows down a lot. Therefore, we assign them to a group and free them
  // every 100th texture.
  if (rtt_count_ % 100 == 0) {
    Ogre::ResourceGroupManager::getSingleton().clearResourceGroup(
        kSubmapTexturesGroup);
  }
  rttTexture_ =
      Ogre::Root::getSingletonPtr()->getTextureManager()->createManual(
          kSubmapsRttPrefix + std::to_string(rtt_count_), kSubmapTexturesGroup,
          Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_RG8,
          Ogre::TU_RENDERTARGET);
  rtt_count_++;

  Ogre::Pass* rtt_pass = submap_scene_material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* const rtt_tex_unit =
      rtt_pass->getNumTextureUnitStates() > 0
          ? rtt_pass->getTextureUnitState(0)
          : rtt_pass->createTextureUnitState();
  rtt_tex_unit->setTexture(rttTexture_);

  Ogre::RenderTexture* const renderTexture =
      rttTexture_->getBuffer()->getRenderTarget();
  renderTexture->addViewport(submaps_scene_camera_)
      ->setBackgroundColour(Ogre::ColourValue(0.5f, 0.f, 0.f));
  {
    ::cartographer::common::MutexLocker locker(&mutex_);
    // TODO(pedrofernandez): Add support for more than one trajectory.
    for (int i = 0; i < trajectories_.front()->Size(); ++i) {
      trajectories_.front()->Get(i).Transform(ros::Time());
      try {
        const ::geometry_msgs::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform(map_frame_property_->getStdString(),
                                       tracking_frame_property_->getStdString(),
                                       ros::Time(0) /* latest */);
        trajectories_.front()->Get(i).SetAlpha(
            transform_stamped.transform.translation.z);
      } catch (tf2::TransformException& e) {
        ROS_WARN("Could not compute submap fading: %s", e.what());
      }
    }
  }
  Ogre::Camera* const actual_camera =
      scene_manager_->getCurrentViewport()->getCamera();
  submaps_scene_camera_->synchroniseBaseSettingsWith(actual_camera);
  submaps_scene_camera_->setCustomProjectionMatrix(
      true, actual_camera->getProjectionMatrix());
  renderTexture->update();

  Ogre::Pass* const pass = screen_blit_material_->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBF_SOURCE_ALPHA,
                         Ogre::SBF_ONE_MINUS_SOURCE_ALPHA);
  Ogre::TextureUnitState* const tex_unit = pass->getNumTextureUnitStates() > 0
                                               ? pass->getTextureUnitState(0)
                                               : pass->createTextureUnitState();
  tex_unit->setTextureName(rttTexture_->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  overlay_->show();
}

}  // namespace rviz
}  // namespace cartographer_ros

PLUGINLIB_EXPORT_CLASS(cartographer_ros::rviz::SubmapsDisplay, ::rviz::Display)
