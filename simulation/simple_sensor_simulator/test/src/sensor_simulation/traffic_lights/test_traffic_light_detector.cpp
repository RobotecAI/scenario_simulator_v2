// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include "test_traffic_light_detector.hpp"

auto makeUpdateTrafficLightsRequest(const int32_t id, const TrafficLightColor color)
  -> simulation_api_schema::UpdateTrafficLightsRequest
{
  simulation_api_schema::UpdateTrafficLightsRequest request;
  auto * signal = request.add_states();
  signal->set_id(id);
  auto * traffic_light = signal->add_traffic_light_status();
  traffic_light->set_color(color);
  return request;
}

/**
 * @note Test basic functionality. Test updating frame correctness with a vector containing some
 * traffic light state - the goal is to verify that correct data is published on correct topic.
 */
TEST_F(TrafficLightDetectorTest_AutoPerceptionMsgs, updateFrame_correctState)
{
  traffic_lights_detector_auto_->updateFrame(
    stamp_, makeUpdateTrafficLightsRequest(
              static_cast<int32_t>(34802), TrafficLightColor::TrafficLight_Color_RED));
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_auto_, nullptr);
  EXPECT_EQ(received_msg_auto_->header.frame_id, "camera_link");
  EXPECT_EQ(received_msg_auto_->signals.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_auto_->signals[0].map_primitive_id, 34802);
  EXPECT_EQ(received_msg_auto_->signals[0].lights.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_auto_->signals[0].lights[0].color, red_light_);
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(TrafficLightDetectorTest_AutoPerceptionMsgs, updateFrame_emptyVector)
{
  traffic_lights_detector_auto_->updateFrame(
    stamp_, simulation_api_schema::UpdateTrafficLightsRequest());
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_auto_, nullptr);
  EXPECT_EQ(received_msg_auto_->header.frame_id, "camera_link");
  EXPECT_EQ(received_msg_auto_->signals.size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality. Test updating frame correctness with a vector containing some
 * traffic light state - the goal is to verify that correct data is published on correct topic.
 */
TEST_F(TrafficLightDetectorTest_PerceptionMsgs, updateFrame_correctState)
{
  traffic_lights_detector_perception_->updateFrame(
    stamp_, makeUpdateTrafficLightsRequest(
              static_cast<int32_t>(34802), TrafficLightColor::TrafficLight_Color_RED));
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_perception_, nullptr);
  EXPECT_EQ(received_msg_perception_->signals.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_perception_->signals[0].traffic_signal_id, 34806);
  EXPECT_EQ(received_msg_perception_->signals[0].elements.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_perception_->signals[0].elements[0].color, red_light_);
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(TrafficLightDetectorTest_PerceptionMsgs, updateFrame_emptyVector)
{
  traffic_lights_detector_perception_->updateFrame(
    stamp_, simulation_api_schema::UpdateTrafficLightsRequest());
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_perception_, nullptr);
  EXPECT_EQ(received_msg_perception_->signals.size(), static_cast<size_t>(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
