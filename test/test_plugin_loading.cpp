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
#include <pluginlib/class_loader.hpp>
#include "nav2_costmap_2d/layer.hpp"

TEST(PluginLoadingTest, DynamicPluginlibInstantiate)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");

  std::vector<std::string> classes = loader.getDeclaredClasses();
  bool found = false;
  for (const auto & c : classes) {
    if (c == "adaptive_costmap_2d::AdaptiveCostmapLayer") {
      found = true;
      break;
    }
  }

  EXPECT_TRUE(found) << "Plugin 'adaptive_costmap_2d::AdaptiveCostmapLayer' not declared in pluginlib manifest!";

  std::shared_ptr<nav2_costmap_2d::Layer> layer;
  EXPECT_NO_THROW({
    layer = loader.createSharedInstance("adaptive_costmap_2d::AdaptiveCostmapLayer");
  });

  ASSERT_NE(layer, nullptr);
  EXPECT_TRUE(layer->isClearable());
}
