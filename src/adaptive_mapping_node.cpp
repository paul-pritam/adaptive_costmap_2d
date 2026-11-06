// Copyright 2026 Pritam Paul
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

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "adaptive_costmap_2d/sparse_grid_2d.hpp"
#include "adaptive_costmap_2d/block_visualizer.hpp"

namespace adaptive_costmap_2d
{

class AdaptiveMappingNode : public rclcpp::Node
{
public:
  AdaptiveMappingNode()
  : Node("adaptive_mapping_node"),
    sparse_grid_(0.05, 0.0, 0.0, 255) // Default: 255 (unknown)
  {
    declare_parameter("scan_topic", std::string("/bcr_bot_1/scan"));
    declare_parameter("global_frame", std::string("map"));
    declare_parameter("resolution", 0.05);
    declare_parameter("publish_rate", 2.0); // 2 Hz

    scan_topic_ = get_parameter("scan_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    resolution_ = get_parameter("resolution").as_double();
    const double publish_rate = get_parameter("publish_rate").as_double();

    sparse_grid_ = SparseGrid2D<16>(resolution_, 0.0, 0.0, 255);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AdaptiveMappingNode::scan_callback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/adaptive_map/map", rclcpp::QoS(1).transient_local());

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/adaptive_map/sparse_blocks", rclcpp::QoS(5));

    const auto timer_period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(0.1, publish_rate)));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&AdaptiveMappingNode::publish_map_and_markers, this));

    RCLCPP_INFO(
      get_logger(),
      "AdaptiveMappingNode online. Subscribing to '%s', global frame '%s', resolution %.3fm",
      scan_topic_.c_str(), global_frame_.c_str(), resolution_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id,
        tf2_ros::fromMsg(msg->header.stamp),
        tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      try {
        tf_stamped = tf_buffer_->lookupTransform(
          global_frame_, msg->header.frame_id,
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex2) {
        return;
      }
    }

    const double origin_x = tf_stamped.transform.translation.x;
    const double origin_y = tf_stamped.transform.translation.y;

    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::lock_guard<std::mutex> lock(grid_mutex_);

    int x0, y0;
    sparse_grid_.world_to_cell(origin_x, origin_y, x0, y0);

    const double angle_min = msg->angle_min;
    const double angle_inc = msg->angle_increment;
    const double range_min = msg->range_min;
    const double range_max = msg->range_max;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (std::isnan(r) || r < range_min) {
        continue;
      }

      const double angle = yaw + angle_min + static_cast<double>(i) * angle_inc;
      const bool is_hit = (r < range_max);
      const double valid_r = is_hit ? r : range_max;

      const double hit_x = origin_x + valid_r * std::cos(angle);
      const double hit_y = origin_y + valid_r * std::sin(angle);

      int x1, y1;
      sparse_grid_.world_to_cell(hit_x, hit_y, x1, y1);

      // Hierarchical DDA raycasting clears free space
      sparse_grid_.raytrace(x0, y0, x1, y1, [this](int cx, int cy) {
        // Free space = cost 0
        this->sparse_grid_.set_cost(cx, cy, 0);
      });

      // Mark lethal obstacle at endpoint (cost 254)
      if (is_hit) {
        sparse_grid_.set_cost(x1, y1, 254);
      }
    }
  }

  void publish_map_and_markers()
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);

    if (sparse_grid_.allocated_blocks_count() == 0) {
      return;
    }

    const auto stamp = this->now();

    // 1. Publish Sparse Block Visualization Markers
    const int w = sparse_grid_.max_cell_x() - sparse_grid_.min_cell_x() + 1;
    const int h = sparse_grid_.max_cell_y() - sparse_grid_.min_cell_y() + 1;
    auto markers = BlockVisualizer::create_markers(
      sparse_grid_, global_frame_, stamp, w, h);
    marker_pub_->publish(markers);

    // 2. Publish OccupancyGrid
    const int min_x = sparse_grid_.min_cell_x();
    const int min_y = sparse_grid_.min_cell_y();
    const int max_x = sparse_grid_.max_cell_x();
    const int max_y = sparse_grid_.max_cell_y();

      if (min_x > max_x || min_y > max_y) {
        return;
      }

      const uint32_t width = static_cast<uint32_t>(max_x - min_x + 1);
      const uint32_t height = static_cast<uint32_t>(max_y - min_y + 1);

      nav_msgs::msg::OccupancyGrid map_msg;
      map_msg.header.frame_id = global_frame_;
      map_msg.header.stamp = stamp;

      map_msg.info.resolution = static_cast<float>(resolution_);
      map_msg.info.width = width;
      map_msg.info.height = height;

      double origin_wx, origin_wy;
      sparse_grid_.cell_to_world(min_x, min_y, origin_wx, origin_wy);
      map_msg.info.origin.position.x = origin_wx - 0.5 * resolution_;
      map_msg.info.origin.position.y = origin_wy - 0.5 * resolution_;
      map_msg.info.origin.position.z = 0.0;
      map_msg.info.origin.orientation.w = 1.0;

      map_msg.data.resize(width * height, -1);

      for (uint32_t j = 0; j < height; ++j) {
        const int cy = min_y + static_cast<int>(j);
        for (uint32_t i = 0; i < width; ++i) {
          const int cx = min_x + static_cast<int>(i);
          const uint8_t cost = sparse_grid_.get_cost(cx, cy);

          int8_t occ_val = -1;
          if (cost == 0) {
            occ_val = 0; // Free
          } else if (cost == 254) {
            occ_val = 100; // Lethal obstacle
          } else if (cost == 255) {
            occ_val = -1; // Unknown / unexplored
          } else {
            occ_val = static_cast<int8_t>((static_cast<int>(cost) * 100) / 254);
          }
          map_msg.data[j * width + i] = occ_val;
        }
      }

      map_pub_->publish(map_msg);
  }

  SparseGrid2D<16> sparse_grid_;
  std::mutex grid_mutex_;

  std::string scan_topic_{"/bcr_bot_1/scan"};
  std::string global_frame_{"map"};
  double resolution_{0.05};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adaptive_costmap_2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adaptive_costmap_2d::AdaptiveMappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
