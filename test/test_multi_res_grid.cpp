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

#include <gtest/gtest.h>
#include "adaptive_costmap_2d/multi_res_quadtree.hpp"
#include "adaptive_costmap_2d/multi_res_grid_2d.hpp"
#include "adaptive_costmap_2d/block_visualizer.hpp"

using namespace adaptive_costmap_2d;

TEST(MultiResQuadTreeTest, NodeSubdivideAndPrune)
{
  QuadNode node(2, 255); // Depth 2: 4x4 cells, initial cost 255
  EXPECT_TRUE(node.is_leaf);
  EXPECT_EQ(node.cost, 255);

  node.subdivide();
  EXPECT_FALSE(node.is_leaf);
  ASSERT_NE(node.children, nullptr);
  EXPECT_EQ((*node.children)[0].depth, 1);
  EXPECT_EQ((*node.children)[0].cost, 255);

  // Set all 4 children to cost 0 (free space)
  for (int i = 0; i < 4; ++i) {
    (*node.children)[i].cost = 0;
    (*node.children)[i].has_lethal = false;
  }

  // Prune should successfully collapse them into a single coarse leaf
  EXPECT_TRUE(node.try_prune());
  EXPECT_TRUE(node.is_leaf);
  EXPECT_EQ(node.cost, 0);
  EXPECT_EQ(node.children, nullptr);
}

TEST(MultiResGridTest, CoarseToFineAdaptiveSubdivision)
{
  MultiResGrid2D<4> grid(0.05, 0.0, 0.0, 255); // Depth 4 = 16x16 cells = 0.8m

  // Clear entire block (0, 0) as free space (cost 0)
  for (int y = 0; y < 16; ++y) {
    for (int x = 0; x < 16; ++x) {
      grid.set_cost(x, y, 0);
    }
  }

  // Because all cells are free, it should collapse into a single coarse leaf!
  std::vector<MultiResBox> boxes;
  grid.collect_multires_boxes(boxes);
  ASSERT_EQ(boxes.size(), 1UL);
  EXPECT_EQ(boxes[0].depth, 4); // Root depth
  EXPECT_NEAR(boxes[0].size_m, 0.80, 1e-4); // Coarse 0.8m box
  EXPECT_EQ(boxes[0].cost, 0);

  // Now mark a lethal obstacle cell at (5, 5)
  grid.set_cost(5, 5, 254);

  // Grid should now dynamically subdivide near the obstacle!
  grid.collect_multires_boxes(boxes);
  EXPECT_GT(boxes.size(), 1UL); // Now divided into multiple LOD boxes!

  bool found_coarse = false;
  bool found_fine = false;
  for (const auto & b : boxes) {
    if (b.depth >= 2 && b.cost == 0) {
      found_coarse = true; // Large free boxes remain in open space
    }
    if (b.depth == 0 && b.cost == 254) {
      found_fine = true;   // Exact 0.05m fine leaf at obstacle cell
    }
  }
  EXPECT_TRUE(found_coarse);
  EXPECT_TRUE(found_fine);

  // Coordinate retrieval must remain 100% exact
  EXPECT_EQ(grid.get_cost(5, 5), 254);
  EXPECT_EQ(grid.get_cost(0, 0), 0);
  EXPECT_EQ(grid.get_cost(10, 10), 0);
  EXPECT_EQ(grid.get_cost(100, 100), 255); // Unexplored sentinel
}

TEST(MultiResGridTest, EphemeralDecayCollapsesBackToCoarse)
{
  MultiResGrid2D<4> grid(0.05, 0.0, 0.0, 255);

  // Initialize block (0, 0) as free space
  for (int y = 0; y < 16; ++y) {
    for (int x = 0; x < 16; ++x) {
      grid.set_cost(x, y, 0);
    }
  }

  // Add temporary transient obstacle at t=10.0
  grid.set_cost(2, 2, 254, 10.0);

  std::vector<MultiResBox> boxes;
  grid.collect_multires_boxes(boxes);
  EXPECT_GT(boxes.size(), 1UL); // Subdivided

  // Run decay after 10.0s (t=20.0 with 5.0s expiry)
  size_t decayed = grid.decay_ephemeral_obstacles(20.0, 5.0, 4);
  EXPECT_GT(decayed, 0UL);
  EXPECT_EQ(grid.get_cost(2, 2), 0); // Decayed to free

  // The quadtree should automatically collapse back into coarse free node!
  grid.collect_multires_boxes(boxes);
  ASSERT_EQ(boxes.size(), 1UL);
  EXPECT_EQ(boxes[0].depth, 4); // Re-collapsed to 0.8m coarse box!
  EXPECT_EQ(boxes[0].cost, 0);
}

TEST(MultiResGridTest, MultiResVisualizerMarkerGeneration)
{
  MultiResGrid2D<4> grid(0.05, 0.0, 0.0, 255);

  grid.set_cost(0, 0, 0);
  grid.set_cost(5, 5, 254);

  auto markers = BlockVisualizer::create_multires_markers(grid, "map", rclcpp::Time(0), 100, 100, 0.012);

  ASSERT_EQ(markers.markers.size(), 3UL);
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::msg::Marker::POINTS);
  EXPECT_EQ(markers.markers[2].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_NE(markers.markers[2].text.find("MULTI-RESOLUTION OCTOGRID"), std::string::npos);
}
