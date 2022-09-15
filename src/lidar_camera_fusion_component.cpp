// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Headers in this package
#include "lidar_camera_fusion/lidar_camera_fusion_component.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

// Headers needed in this component
#include <chrono>

namespace lidar_camera_fusion
{
LidarCameraFusionComponent::LidarCameraFusionComponent(const rclcpp::NodeOptions & options)
: Node("lidar_camera_fusion_node", options)
{
  // Specify the topic names from args
  std::string camera_topic;
  declare_parameter("camera_topic", "/detection/camera");
  get_parameter("camera_topic", camera_topic);

  std::string lidar_topic;
  declare_parameter("lidar_topic", "/detection/lidar");
  get_parameter("lidar_topic", lidar_topic);

  int duration_msec;
  declare_parameter("fusion_poll_period_msec", 100);
  get_parameter("fusion_poll_period_msec", duration_msec);
  int delay;
  declare_parameter("fusion_allowed_delay_msec", 50);
  get_parameter("fusion_allowed_delay_msec", delay);


//  std::shared_ptr<Sync2T> temp = std::make_shared(
//        Sync2T(this, { camera_topic, lidar_topic }, std::chrono::milliseconds{duration_msec}, std::chrono::milliseconds{delay} )
//        );

//  this->sync_camera_lidar_ = temp;



}



}  // namespace lidar_camera_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_camera_fusion::LidarCameraFusionComponent)
