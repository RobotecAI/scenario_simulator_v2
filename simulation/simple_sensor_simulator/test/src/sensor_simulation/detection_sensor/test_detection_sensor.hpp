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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_DETECTION_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_DETECTION_SENSOR_HPP_

#include <gtest/gtest.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <string>
#include <vector>

#include "../../utils/helper_functions.hpp"

using namespace simple_sensor_simulator;

using DetectedObjectsMsg = autoware_auto_perception_msgs::msg::DetectedObjects;
using TrackedObjectsMsg = autoware_auto_perception_msgs::msg::TrackedObjects;
using ObjectClassification = autoware_auto_perception_msgs::msg::ObjectClassification;

class DetectionSensorTest : public ::testing::Test
{
protected:
  DetectionSensorTest()
  : config_(utils::constructDetectionSensorConfiguration("ego", "awf/universe", 0.1, 100, true)),
    entity_pose_(utils::makePose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
    entity_dimensions_(utils::makeDimensions(4.5, 2.0, 1.5))
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("detection_sensor_test_node");

    makeRosInterface();
    makeEgo();

    detection_sensor_ = std::make_unique<DetectionSensor<DetectedObjectsMsg>>(
      0.0, config_, detected_objects_publisher_, ground_truth_objects_publisher_);
  }
  ~DetectionSensorTest() { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<DetectedObjectsMsg>::SharedPtr detected_objects_publisher_;
  rclcpp::Publisher<TrackedObjectsMsg>::SharedPtr ground_truth_objects_publisher_;
  rclcpp::Subscription<DetectedObjectsMsg>::SharedPtr detected_objects_subscriber_;

  std::vector<EntityStatus> status_;
  std::vector<std::string> lidar_detected_entities_;

  simulation_api_schema::DetectionSensorConfiguration config_;
  std::unique_ptr<DetectionSensorBase> detection_sensor_;
  DetectedObjectsMsg::SharedPtr received_msg_;

  double current_simulation_time_{1.0};
  rclcpp::Time current_ros_time_{1};
  geometry_msgs::msg::Pose entity_pose_;
  geometry_msgs::msg::Vector3 entity_dimensions_;

  auto initializeEntityStatus(
    const std::string & name, const EntityType::Enum type, const EntitySubtype::Enum subtype)
    -> void
  {
    const auto entity_status =
      utils::makeEntity(name, type, subtype, entity_pose_, entity_dimensions_);
    status_.push_back(entity_status);
    lidar_detected_entities_.push_back(name);
  }

private:
  auto makeEgo() -> void
  {
    const auto ego_status = utils::makeEntity(
      "ego", EntityType::EGO, EntitySubtype::CAR,
      utils::makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), entity_dimensions_);
    status_.push_back(ego_status);
  }

  auto makeRosInterface() -> void
  {
    detected_objects_publisher_ =
      node_->create_publisher<DetectedObjectsMsg>("detected_objects_output", 10);
    ground_truth_objects_publisher_ =
      node_->create_publisher<TrackedObjectsMsg>("tracked_objects_output", 10);

    detected_objects_subscriber_ = node_->create_subscription<DetectedObjectsMsg>(
      "detected_objects_output", 10,
      [this](const DetectedObjectsMsg::SharedPtr msg) { received_msg_ = msg; });
  }
};

struct DetectionTestParam
{
  std::string entity_name_;
  EntityType::Enum entity_type_;
  EntitySubtype::Enum entity_subtype_;
  int expected_label_;
};

class DetectionSensorTestParameterized : public DetectionSensorTest,
                                         public ::testing::WithParamInterface<DetectionTestParam>
{
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_DETECTION_SENSOR_HPP_
