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
#include <vision_msgs/msg/bounding_box2_d.hpp>

// Headers needed in this component
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <chrono>

#include "lidar_camera_fusion/hungarian.hpp"

namespace lidar_camera_fusion
{
LidarCameraFusionComponent::LidarCameraFusionComponent(const rclcpp::NodeOptions & options)
: Node("lidar_camera_fusion_node", options)
{
  pub_ =
    this->create_publisher<perception_msgs::msg::Detection3DArray>("detection/fusion_result", 1);

  // Specify the topic names from args
  std::string camera_topic;
  declare_parameter("camera_topic", "detection/camera");
  get_parameter("camera_topic", camera_topic);

  std::string lidar_topic;
  declare_parameter("lidar_topic", "detection/lidar");
  get_parameter("lidar_topic", lidar_topic);

  int duration_msec;
  declare_parameter("fusion_poll_period_msec", 100);
  get_parameter("fusion_poll_period_msec", duration_msec);

  int delay;
  declare_parameter("fusion_allowed_delay_msec", 50);
  get_parameter("fusion_allowed_delay_msec", delay);

  declare_parameter("iou_lower_bound", 0.5);
  get_parameter("iou_lower_bound", iou_lower_bound);

  sync_camera_lidar_ = std::shared_ptr<Sync2T>(new Sync2T(
    this, {camera_topic, lidar_topic}, std::chrono::milliseconds{duration_msec},
    std::chrono::milliseconds{delay}));

  sync_camera_lidar_->registerCallback(std::bind(
    &LidarCameraFusionComponent::callback, this, std::placeholders::_1, std::placeholders::_2));
}

double LidarCameraFusionComponent::getIoU(
  const vision_msgs::msg::BoundingBox2D & a, const vision_msgs::msg::BoundingBox2D & b)
{
  typedef boost::geometry::model::d2::point_xy<double> point;
  typedef boost::geometry::model::polygon<point> polygon;

  polygon poly_a;
  boost::geometry::exterior_ring(poly_a) =
    boost::assign::list_of<point>(a.center.x - a.size_x / 2.0, a.center.y - a.size_y / 2.0)(
      a.center.x - a.size_x / 2.0, a.center.y + a.size_y / 2.0)(
      a.center.x + a.size_x / 2.0, a.center.y + a.size_y / 2.0)(
      a.center.x + a.size_x / 2.0, a.center.y - a.size_y / 2.0)(
      a.center.x - a.size_x / 2.0, a.center.y - a.size_y / 2.0);

  polygon poly_b;
  boost::geometry::exterior_ring(poly_b) =
    boost::assign::list_of<point>(b.center.x - b.size_x / 2.0, b.center.y - b.size_y / 2.0)(
      b.center.x - b.size_x / 2.0, b.center.y + b.size_y / 2.0)(
      b.center.x + b.size_x / 2.0, b.center.y + b.size_y / 2.0)(
      b.center.x + b.size_x / 2.0, b.center.y - b.size_y / 2.0)(
      b.center.x - b.size_x / 2.0, b.center.y - b.size_y / 2.0);
  if (
    boost::geometry::intersects(poly_a, poly_b) || boost::geometry::within(poly_a, poly_b) ||
    boost::geometry::within(poly_b, poly_a)) {
    std::vector<polygon> union_poly, intersection;
    boost::geometry::union_(poly_a, poly_b, union_poly);
    boost::geometry::intersection(poly_a, poly_b, intersection);
    if (intersection.size() == 1 && union_poly.size() == 1) {
      return boost::geometry::area(intersection[0]) / boost::geometry::area(union_poly[0]);
    }
  }
  return 0;
}

void LidarCameraFusionComponent::callback(CallbackT camera, CallbackT lidar)
{
  if (!camera || !lidar) {
    return;
  }

  // Hungarian matching solver.
  HungarianAlgorithm solver;

  // Allocate 2-dim array in advance.
  std::vector<std::vector<double>> iou_matrix;
  size_t m = lidar.value()->detections.size();
  size_t n = camera.value()->detections.size();

  // See: https://stackoverflow.com/questions/15889578/how-can-i-resize-a-2d-vector-of-objects-given-the-width-and-height
  iou_matrix.resize(m, std::vector<double>(n));

  // Calculate all the IoUs.
  // The number of the items should be m * n.
  auto lidar_det = lidar.value()->detections;
  auto camera_det = camera.value()->detections;

  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      iou_matrix[i][j] = getIoU(lidar_det[i].bbox, camera_det[j].bbox);
    }
  }
  std::vector<int> assignments;
  solver.Solve(iou_matrix, assignments);

  // Compose a message
  perception_msgs::msg::Detection3DArray detection_3d_array;
  detection_3d_array.header = lidar.value()->header;
  // Filter detections
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t j = assignments[i];

    if (!lidar_det[i].bbox_3d.empty()) {
      if (iou_matrix[i][j] >= iou_lower_bound) {
        perception_msgs::msg::Detection3D detection3d;
        detection3d.header = lidar_det[i].header;
        detection3d.label = camera_det[j].label;
        detection3d.score = camera_det[j].score;
        detection3d.detection_id = lidar_det[i].detection_id;
        detection3d.bbox = lidar_det[i].bbox_3d[0];
        detection_3d_array.detections.emplace_back(detection3d);
      }
    }
  }

  pub_->publish(detection_3d_array);
}

}  // namespace lidar_camera_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_camera_fusion::LidarCameraFusionComponent)
