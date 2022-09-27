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

#pragma once

#include "lidar_camera_fusion/visibility_control.h"

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>

// Headers needed in pub/sub, exposed types
#include <memory>  // shared_ptr in pub_
#include <message_synchronizer/message_synchronizer.hpp>
#include <perception_msgs/msg/detection2_d_array.hpp>
#include <perception_msgs/msg/detection3_d_array.hpp>

namespace lidar_camera_fusion
{
typedef message_synchronizer::MessageSynchronizer2<
  perception_msgs::msg::Detection2DArray, perception_msgs::msg::Detection2DArray>
  Sync2T;
typedef const boost::optional<const perception_msgs::msg::Detection2DArray::SharedPtr> & CallbackT;

class LidarCameraFusionComponent : public rclcpp::Node
{
public:
  lidar_camera_fusion_PUBLIC explicit LidarCameraFusionComponent(
    const rclcpp::NodeOptions & options);

private:
  double getIoU(
    const vision_msgs::msg::BoundingBox2D & a, const vision_msgs::msg::BoundingBox2D & b);
  std::shared_ptr<rclcpp::Publisher<perception_msgs::msg::Detection3DArray>> pub_;
  std::shared_ptr<Sync2T> sync_camera_lidar_;
  void callback(CallbackT camera, CallbackT lidar);

  double iou_lower_bound;
};
}  // namespace lidar_camera_fusion
