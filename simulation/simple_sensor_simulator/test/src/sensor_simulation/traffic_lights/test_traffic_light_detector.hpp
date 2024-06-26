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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_TRAFFIC_LIGHT_DETECTOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_TRAFFIC_LIGHT_DETECTOR_HPP_

#include <gtest/gtest.h>
#include <simulation_api_schema.pb.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/traffic_lights/traffic_lights_detector.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <vector>

using namespace simple_sensor_simulator;

using AutoPerceptionTrafficSignalArray = autoware_auto_perception_msgs::msg::TrafficSignalArray;
using PerceptionTrafficSignalArray = autoware_perception_msgs::msg::TrafficSignalArray;
using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;
using TrafficLightColor = simulation_api_schema::TrafficLight_Color;

class TrafficLightDetectorTestBase : public ::testing::Test
{
protected:
  TrafficLightDetectorTestBase()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("traffic_light_detector_test_node");

    hdmap_utils_ = std::make_shared<hdmap_utils::HdMapUtils>(
      ament_index_cpp::get_package_share_directory("simple_sensor_simulator") +
        "/map/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0));
  }

  ~TrafficLightDetectorTestBase() { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  const uint8_t red_light_{TrafficSignalElement::RED};
  const rclcpp::Time stamp_{0};
};

// awf/universe
class TrafficLightDetectorTest_AutoPerceptionMsgs : public TrafficLightDetectorTestBase
{
protected:
  TrafficLightDetectorTest_AutoPerceptionMsgs()
  {
    publisher_auto_ =
      std::make_shared<traffic_simulator::TrafficLightPublisher<AutoPerceptionTrafficSignalArray>>(
        perception_msgs_topic_, node_, hdmap_utils_);
    subscription_auto_ = node_->create_subscription<AutoPerceptionTrafficSignalArray>(
      perception_msgs_topic_, 10,
      [this](const AutoPerceptionTrafficSignalArray::SharedPtr msg) { received_msg_auto_ = msg; });

    traffic_lights_detector_auto_ =
      std::make_unique<traffic_lights::TrafficLightsDetector>(publisher_auto_);
  }

  const std::string perception_msgs_topic_ = "traffic_light_detector_output_perception";

  std::shared_ptr<traffic_simulator::TrafficLightPublisherBase> publisher_auto_;
  std::unique_ptr<traffic_lights::TrafficLightsDetector> traffic_lights_detector_auto_;
  rclcpp::Subscription<AutoPerceptionTrafficSignalArray>::SharedPtr subscription_auto_;
  AutoPerceptionTrafficSignalArray::SharedPtr received_msg_auto_;
};

// awf/universe/20230906
class TrafficLightDetectorTest_PerceptionMsgs : public TrafficLightDetectorTestBase
{
protected:
  TrafficLightDetectorTest_PerceptionMsgs()
  {
    publisher_perception_ =
      std::make_shared<traffic_simulator::TrafficLightPublisher<PerceptionTrafficSignalArray>>(
        auto_perception_msgs_topic_, node_, hdmap_utils_);
    subscription_perception_ = node_->create_subscription<PerceptionTrafficSignalArray>(
      auto_perception_msgs_topic_, 10, [this](const PerceptionTrafficSignalArray::SharedPtr msg) {
        received_msg_perception_ = msg;
      });

    traffic_lights_detector_perception_ =
      std::make_unique<traffic_lights::TrafficLightsDetector>(publisher_perception_);
  }

  const std::string auto_perception_msgs_topic_ = "traffic_light_detector_output_auto";

  std::shared_ptr<traffic_simulator::TrafficLightPublisherBase> publisher_perception_;
  std::unique_ptr<traffic_lights::TrafficLightsDetector> traffic_lights_detector_perception_;
  rclcpp::Subscription<PerceptionTrafficSignalArray>::SharedPtr subscription_perception_;
  PerceptionTrafficSignalArray::SharedPtr received_msg_perception_;
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_TRAFFIC_LIGHT_DETECTOR_HPP_
