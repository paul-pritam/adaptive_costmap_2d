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

#include "adaptive_costmap_2d/adaptive_costmap_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(adaptive_costmap_2d::AdaptiveCostmapLayer, nav2_costmap_2d::Layer)

namespace adaptive_costmap_2d
{

AdaptiveCostmapLayer::AdaptiveCostmapLayer()
: sparse_grid_(0.05, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION)
{
}

void AdaptiveCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to acquire lifecycle node in AdaptiveCostmapLayer");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("scan")));
  declareParameter("track_unknown_space", rclcpp::ParameterValue(false));
  declareParameter("combination_method", rclcpp::ParameterValue(1));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", scan_topic_);
  node->get_parameter(name_ + ".track_unknown_space", track_unknown_space_);
  node->get_parameter(name_ + ".combination_method", combination_method_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  const double res = layered_costmap_->getCostmap()->getResolution();
  const double ox = layered_costmap_->getCostmap()->getOriginX();
  const double oy = layered_costmap_->getCostmap()->getOriginY();
  sparse_grid_ = SparseGrid2D<16>(res, ox, oy, nav2_costmap_2d::NO_INFORMATION);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AdaptiveCostmapLayer::laser_scan_callback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "Initialized AdaptiveCostmapLayer '%s' subscribing to '%s' (resolution=%.3fm)",
    name_.c_str(), scan_topic_.c_str(), res);
}

void AdaptiveCostmapLayer::matchSize()
{
  std::lock_guard<std::recursive_mutex> lock(grid_mutex_);
  if (!layered_costmap_) {
    return;
  }
  const auto * costmap = layered_costmap_->getCostmap();
  sparse_grid_ = SparseGrid2D<16>(
    costmap->getResolution(),
    costmap->getOriginX(),
    costmap->getOriginY(),
    nav2_costmap_2d::NO_INFORMATION);
}

void AdaptiveCostmapLayer::reset()
{
  std::lock_guard<std::recursive_mutex> lock(grid_mutex_);
  sparse_grid_.clear();
  current_ = true;
}

void AdaptiveCostmapLayer::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_->lookupTransform(
      global_frame_, msg->header.frame_id,
      tf2_ros::fromMsg(msg->header.stamp),
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    // Retry with latest available transform if timestamped lookup fails
    try {
      tf_stamped = tf_->lookupTransform(
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

  std::lock_guard<std::recursive_mutex> lock(grid_mutex_);

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

    // Hierarchical DDA raycasting clears free space along ray
    sparse_grid_.raytrace(x0, y0, x1, y1, [this](int cx, int cy) {
      this->sparse_grid_.set_cost(cx, cy, nav2_costmap_2d::FREE_SPACE);
    });

    // Mark lethal obstacle at endpoint
    if (is_hit) {
      sparse_grid_.set_cost(x1, y1, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  need_bounds_update_ = true;

  auto node = node_.lock();
  if (node) {
    const size_t alloc_blocks = sparse_grid_.allocated_blocks_count();
    const size_t sparse_kb = sparse_grid_.memory_bytes_used() / 1024;
    const auto * master = layered_costmap_->getCostmap();
    const size_t dense_kb = (master->getSizeInCellsX() * master->getSizeInCellsY()) / 1024;
    const double savings = (dense_kb > 0 && dense_kb > sparse_kb)
      ? 100.0 * (1.0 - static_cast<double>(sparse_kb) / static_cast<double>(dense_kb))
      : 0.0;

    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "[AdaptiveCostmap] Blocks: %zu | RAM: %zu KB vs Dense: %zu KB (%.1f%% memory savings)",
      alloc_blocks, sparse_kb, dense_kb, savings);
  }
}

void AdaptiveCostmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(grid_mutex_);
  if (sparse_grid_.allocated_blocks_count() == 0) {
    return;
  }

  double s_min_x, s_min_y, s_max_x, s_max_y;
  sparse_grid_.get_world_bounds(s_min_x, s_min_y, s_max_x, s_max_y);

  *min_x = std::min(*min_x, s_min_x);
  *min_y = std::min(*min_y, s_min_y);
  *max_x = std::max(*max_x, s_max_x);
  *max_y = std::max(*max_y, s_max_y);
}

void AdaptiveCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(grid_mutex_);

  const int bound_min_i = std::max(0, min_i);
  const int bound_min_j = std::max(0, min_j);
  const int bound_max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  const int bound_max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);

  for (int j = bound_min_j; j < bound_max_j; ++j) {
    for (int i = bound_min_i; i < bound_max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      const uint8_t sparse_cost = sparse_grid_.get_cost_world(wx, wy);

      if (sparse_cost == nav2_costmap_2d::NO_INFORMATION && !track_unknown_space_) {
        continue;
      }

      const uint8_t old_cost = master_grid.getCost(i, j);
      if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(i, j, sparse_cost);
      } else if (combination_method_ == 1) { // Max
        master_grid.setCost(i, j, std::max(old_cost, sparse_cost));
      } else { // Overwrite
        master_grid.setCost(i, j, sparse_cost);
      }
    }
  }
}

size_t AdaptiveCostmapLayer::get_allocated_blocks() const noexcept
{
  return sparse_grid_.allocated_blocks_count();
}

size_t AdaptiveCostmapLayer::get_memory_bytes() const noexcept
{
  return sparse_grid_.memory_bytes_used();
}

double AdaptiveCostmapLayer::get_memory_savings_ratio() const noexcept
{
  if (!layered_costmap_) {
    return 0.0;
  }
  const auto * master = layered_costmap_->getCostmap();
  return sparse_grid_.compression_ratio(
    0, 0,
    master->getSizeInCellsX() - 1,
    master->getSizeInCellsY() - 1);
}

}  // namespace adaptive_costmap_2d
