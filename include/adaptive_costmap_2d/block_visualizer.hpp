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

#ifndef ADAPTIVE_COSTMAP_2D__BLOCK_VISUALIZER_HPP_
#define ADAPTIVE_COSTMAP_2D__BLOCK_VISUALIZER_HPP_

#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "adaptive_costmap_2d/sparse_grid_2d.hpp"

namespace adaptive_costmap_2d
{

class BlockVisualizer
{
public:
  template <int TileSize = 16>
  static visualization_msgs::msg::MarkerArray create_markers(
    const SparseGrid2D<TileSize> & grid,
    const std::string & frame_id = "map",
    const rclcpp::Time & stamp = rclcpp::Time(0),
    int dense_width_cells = 0,
    int dense_height_cells = 0,
    double wireframe_thickness = 0.012)
  {
    visualization_msgs::msg::MarkerArray array;

    const double res = grid.resolution();
    const double ox = grid.origin_x();
    const double oy = grid.origin_y();
    const double tile_world_size = static_cast<double>(TileSize) * res;

    // 1. Line List Marker for Sparse Block Wireframes
    visualization_msgs::msg::Marker wireframes;
    wireframes.header.frame_id = frame_id;
    wireframes.header.stamp = stamp;
    wireframes.ns = "adaptive_sparse_wireframes";
    wireframes.id = 0;
    wireframes.type = visualization_msgs::msg::Marker::LINE_LIST;
    wireframes.action = visualization_msgs::msg::Marker::ADD;
    wireframes.scale.x = wireframe_thickness; // Crisp wireframe lines

    // 2. Points Marker for Exact Lethal Obstacle Cells
    visualization_msgs::msg::Marker lethal_cells;
    lethal_cells.header.frame_id = frame_id;
    lethal_cells.header.stamp = stamp;
    lethal_cells.ns = "adaptive_lethal_cells";
    lethal_cells.id = 1;
    lethal_cells.type = visualization_msgs::msg::Marker::POINTS;
    lethal_cells.action = visualization_msgs::msg::Marker::ADD;
    lethal_cells.scale.x = res * 0.9;
    lethal_cells.scale.y = res * 0.9;

    std_msgs::msg::ColorRGBA red_point_color;
    red_point_color.r = 1.0f;
    red_point_color.g = 0.15f;
    red_point_color.b = 0.15f;
    red_point_color.a = 0.95f;
    lethal_cells.color = red_point_color;

    // Wireframe Colors
    std_msgs::msg::ColorRGBA free_wire_color;
    free_wire_color.r = 0.0f;
    free_wire_color.g = 0.85f;
    free_wire_color.b = 0.95f;
    free_wire_color.a = 0.40f;

    std_msgs::msg::ColorRGBA obstacle_wire_color;
    obstacle_wire_color.r = 1.0f;
    obstacle_wire_color.g = 0.25f;
    obstacle_wire_color.b = 0.15f;
    obstacle_wire_color.a = 0.75f;

    double sum_x = 0.0;
    double sum_y = 0.0;
    size_t count = 0;

    const auto & blocks_map = grid.blocks();
    for (const auto & [key, block] : blocks_map) {
      int32_t bx, by;
      SparseGrid2D<TileSize>::unpack_key(key, bx, by);

      const double wx_min = ox + static_cast<double>(bx * TileSize) * res;
      const double wy_min = oy + static_cast<double>(by * TileSize) * res;
      const double wx_max = wx_min + tile_world_size;
      const double wy_max = wy_min + tile_world_size;

      const double center_x = (wx_min + wx_max) * 0.5;
      const double center_y = (wy_min + wy_max) * 0.5;
      sum_x += center_x;
      sum_y += center_y;
      count++;

      const bool has_lethal = block->has_lethal();
      const auto & cur_line_color = has_lethal ? obstacle_wire_color : free_wire_color;

      // 4 boundary lines per tile on z = 0.015 (subtle ground height)
      geometry_msgs::msg::Point p0, p1, p2, p3;
      const double z_line = 0.015;
      p0.x = wx_min; p0.y = wy_min; p0.z = z_line;
      p1.x = wx_max; p1.y = wy_min; p1.z = z_line;
      p2.x = wx_max; p2.y = wy_max; p2.z = z_line;
      p3.x = wx_min; p3.y = wy_max; p3.z = z_line;

      // Segment p0 -> p1
      wireframes.points.push_back(p0);
      wireframes.points.push_back(p1);
      wireframes.colors.push_back(cur_line_color);
      wireframes.colors.push_back(cur_line_color);

      // Segment p1 -> p2
      wireframes.points.push_back(p1);
      wireframes.points.push_back(p2);
      wireframes.colors.push_back(cur_line_color);
      wireframes.colors.push_back(cur_line_color);

      // Segment p2 -> p3
      wireframes.points.push_back(p2);
      wireframes.points.push_back(p3);
      wireframes.colors.push_back(cur_line_color);
      wireframes.colors.push_back(cur_line_color);

      // Segment p3 -> p0
      wireframes.points.push_back(p3);
      wireframes.points.push_back(p0);
      wireframes.colors.push_back(cur_line_color);
      wireframes.colors.push_back(cur_line_color);

      // Extract specific lethal cells inside this block
      if (has_lethal) {
        for (int ly = 0; ly < TileSize; ++ly) {
          for (int lx = 0; lx < TileSize; ++lx) {
            if (block->get(lx, ly) == 254) {
              geometry_msgs::msg::Point pt;
              pt.x = wx_min + (static_cast<double>(lx) + 0.5) * res;
              pt.y = wy_min + (static_cast<double>(ly) + 0.5) * res;
              pt.z = 0.02;
              lethal_cells.points.push_back(pt);
            }
          }
        }
      }
    }

    array.markers.push_back(wireframes);
    array.markers.push_back(lethal_cells);

    // 3. Floating 3D Telemetry HUD Marker
    visualization_msgs::msg::Marker hud;
    hud.header.frame_id = frame_id;
    hud.header.stamp = stamp;
    hud.ns = "adaptive_telemetry_hud";
    hud.id = 2;
    hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    hud.action = visualization_msgs::msg::Marker::ADD;

    if (count > 0) {
      hud.pose.position.x = sum_x / static_cast<double>(count);
      hud.pose.position.y = sum_y / static_cast<double>(count);
      hud.pose.position.z = 2.4; // 2.4m above floor
    } else {
      hud.pose.position.x = ox;
      hud.pose.position.y = oy;
      hud.pose.position.z = 2.4;
    }

    hud.scale.z = 0.28; // Text size: 28cm height
    hud.color.r = 1.0f;
    hud.color.g = 0.95f;
    hud.color.b = 0.2f;
    hud.color.a = 1.0f;

    const size_t sparse_bytes = grid.memory_bytes_used();
    const double sparse_kb = static_cast<double>(sparse_bytes) / 1024.0;

    size_t dense_cells = 0;
    if (dense_width_cells > 0 && dense_height_cells > 0) {
      dense_cells = static_cast<size_t>(dense_width_cells) * static_cast<size_t>(dense_height_cells);
    } else if (count > 0) {
      const int w = grid.max_cell_x() - grid.min_cell_x() + 1;
      const int h = grid.max_cell_y() - grid.min_cell_y() + 1;
      dense_cells = static_cast<size_t>(std::max(0, w)) * static_cast<size_t>(std::max(0, h));
    }
    const double dense_kb = static_cast<double>(dense_cells) / 1024.0;
    const double savings = (dense_kb > 0.0 && dense_kb > sparse_kb)
      ? 100.0 * (1.0 - sparse_kb / dense_kb)
      : 0.0;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "=== ADAPTIVE BLOCK-SPARSE 2D MAP ===\n";
    ss << "Allocated Tiles: " << count << " (0.8m x 0.8m) | Sentinels: Active\n";
    ss << "Sparse RAM: " << sparse_kb << " KB vs Dense: " << dense_kb << " KB\n";
    ss << "Active Memory Savings: " << savings << "% | Temporal Dynamic Decay: Active";

    hud.text = ss.str();
    array.markers.push_back(hud);

    return array;
  }
};

}  // namespace adaptive_costmap_2d

#endif  // ADAPTIVE_COSTMAP_2D__BLOCK_VISUALIZER_HPP_
