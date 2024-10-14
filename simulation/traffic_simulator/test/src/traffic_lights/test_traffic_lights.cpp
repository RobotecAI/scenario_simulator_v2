// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <std_msgs/msg/header.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

#include "../expect_eq_macros.hpp"
#include "helper.hpp"

constexpr double timing_eps = 1e-3;
constexpr double frequency_eps = 0.5;

class TrafficLightsTest : public testing::Test
{
public:
  const lanelet::Id id = 34836;
  const lanelet::Id signal_id = 34806;

  const rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("TrafficLightsTest");

  const std::string path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                           "/map/standard_map/lanelet2_map.osm";

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr =
    std::make_shared<hdmap_utils::HdMapUtils>(
      path, geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
              .latitude(35.61836750154)
              .longitude(139.78066608243)
              .altitude(0.0));

  const std::string red_state = stateFromColor("red");
  const std::string yellow_state = "yellow flashing circle";

  std::unique_ptr<traffic_simulator::TrafficLights> lights =
    std::make_unique<traffic_simulator::TrafficLights>(node_ptr, hdmap_utils_ptr, "awf/universe");
};

TEST_F(TrafficLightsTest, isAnyTrafficLightChanged)
{
  EXPECT_TRUE(lights->isAnyTrafficLightChanged());
}

TEST_F(TrafficLightsTest, getConventionalTrafficLights)
{
  {
    this->lights->getConventionalTrafficLights()->setTrafficLightsState(this->id, this->red_state);

    const auto actual_state =
      this->lights->getConventionalTrafficLights()->getTrafficLightsComposedState(this->id);

    EXPECT_EQ(actual_state, this->red_state);
  }
  {
    this->lights->getConventionalTrafficLights()->setTrafficLightsState(
      this->id, this->yellow_state);

    const auto actual_state =
      this->lights->getConventionalTrafficLights()->getTrafficLightsComposedState(this->id);

    EXPECT_EQ(actual_state, this->yellow_state);
  }
}

TEST_F(TrafficLightsTest, getV2ITrafficLights)
{
  {
    this->lights->getV2ITrafficLights()->setTrafficLightsState(this->id, this->red_state);

    const auto actual_state =
      this->lights->getV2ITrafficLights()->getTrafficLightsComposedState(this->id);

    EXPECT_EQ(actual_state, this->red_state);
  }
  {
    this->lights->getV2ITrafficLights()->setTrafficLightsState(this->id, this->yellow_state);

    const auto actual_state =
      this->lights->getV2ITrafficLights()->getTrafficLightsComposedState(this->id);

    EXPECT_EQ(actual_state, this->yellow_state);
  }
}

TEST_F(TrafficLightsTest, startTrafficLightsUpdate)
{
  this->lights->getConventionalTrafficLights()->setTrafficLightsState(this->id, this->red_state);
  this->lights->getV2ITrafficLights()->setTrafficLightsState(this->id, this->red_state);

  std::vector<visualization_msgs::msg::MarkerArray> markers;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscriber =
    this->node_ptr->template create_subscription<visualization_msgs::msg::MarkerArray>(
      "traffic_light/marker", 10,
      [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg_in) {
        markers.push_back(*msg_in);
      });

  this->lights->startTrafficLightsUpdate(20.0, 10.0);

  // start time is required to be measured here and not from first message, because there are two publishers publishing to this topic at the same time
  const auto start_time = node_ptr->now();

  // spin for 1 second
  const auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(1020);
  while (std::chrono::system_clock::now() < end) {
    rclcpp::spin_some(this->node_ptr);
  }

  std::vector<std_msgs::msg::Header> headers;

  // verify
  for (std::size_t i = 0; i < markers.size(); ++i) {
    const auto & one_marker = markers[i].markers;
    EXPECT_EQ(one_marker.size(), static_cast<std::size_t>(1));

    if (
      one_marker.front().header.stamp.sec != 0 and one_marker.front().header.stamp.nanosec != 0u) {
      headers.push_back(one_marker.front().header);
    }
  }

  // verify message timing
  const double expected_frequency = 30.0;
  const double actual_frequency =
    static_cast<double>(headers.size()) /
    static_cast<double>(getTime(headers.back()) - getTime(start_time)) * 1e+9;
  EXPECT_NEAR(actual_frequency, expected_frequency, frequency_eps);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
