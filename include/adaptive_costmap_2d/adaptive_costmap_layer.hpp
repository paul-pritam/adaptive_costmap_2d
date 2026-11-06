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

#ifndef ADAPTIVE_COSTMAP_2D__ADAPTIVE_COSTMAP_LAYER_HPP_
#define ADAPTIVE_COSTMAP_2D__ADAPTIVE_COSTMAP_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "adaptive_costmap_2d/sparse_grid_2d.hpp"
#include "adaptive_costmap_2d/block_visualizer.hpp"

namespace adaptive_costmap_2d
{

class AdaptiveCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  AdaptiveCostmapLayer();
  virtual ~AdaptiveCostmapLayer() override = default;

  virtual void onInitialize() override;

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override;
  virtual bool isClearable() override { return true; }
  virtual void matchSize() override;

  [[nodiscard]] size_t get_allocated_blocks() const noexcept;
  [[nodiscard]] size_t get_memory_bytes() const noexcept;
  [[nodiscard]] double get_memory_savings_ratio() const noexcept;

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  SparseGrid2D<16> sparse_grid_;
  std::recursive_mutex grid_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr block_markers_pub_;
  std::string scan_topic_{"/scan"};
  std::string global_frame_{"map"};

  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};

  bool need_bounds_update_{false};
  bool track_unknown_space_{false};
  int combination_method_{1}; // 1 = Max
};

}  // namespace adaptive_costmap_2d

#endif  // ADAPTIVE_COSTMAP_2D__ADAPTIVE_COSTMAP_LAYER_HPP_
