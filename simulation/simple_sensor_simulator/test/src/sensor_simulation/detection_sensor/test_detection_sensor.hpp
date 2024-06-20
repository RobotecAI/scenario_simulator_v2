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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <vector>

using namespace simple_sensor_simulator;

using DetectedObjectsMsg = autoware_auto_perception_msgs::msg::DetectedObjects;
using TrackedObjectsMsg = autoware_auto_perception_msgs::msg::TrackedObjects;
using ObjectClassification = autoware_auto_perception_msgs::msg::ObjectClassification;
using EntityType = traffic_simulator_msgs::msg::EntityType;
using EntitySubtype = traffic_simulator_msgs::msg::EntitySubtype;
using EntityStatus = traffic_simulator_msgs::EntityStatus;

class DetectionSensorTest : public ::testing::Test
{
protected:
  DetectionSensorTest()
  {
    configureDetectionSensor();

    entity_pose_ = createPose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    entity_dimensions_ = createDimensions(4.5, 2.0, 1.5);

    createEgo();
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("detection_sensor_test_node");

    createRosInterface();

    detection_sensor_ = std::make_unique<DetectionSensor<DetectedObjectsMsg>>(
      0.0, config_, detected_objects_publisher_, ground_truth_objects_publisher_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<DetectedObjectsMsg>::SharedPtr detected_objects_publisher_;
  rclcpp::Publisher<TrackedObjectsMsg>::SharedPtr ground_truth_objects_publisher_;
  rclcpp::Subscription<DetectedObjectsMsg>::SharedPtr detected_objects_subscriber_;

  EntityStatus ego_status_;
  std::vector<EntityStatus> status_;
  std::vector<std::string> lidar_detected_entities_;

  std::unique_ptr<DetectionSensorBase> detection_sensor_;
  simulation_api_schema::DetectionSensorConfiguration config_;
  DetectedObjectsMsg::SharedPtr received_msg_;

  double current_simulation_time_{1.0};
  rclcpp::Time current_ros_time_{1};
  geometry_msgs::msg::Pose entity_pose_;
  geometry_msgs::msg::Vector3 entity_dimensions_;

  auto createEgo() -> void
  {
    initializeEntity(
      ego_status_, "ego", EntityType::EGO, EntitySubtype::CAR,
      createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), entity_dimensions_);
    status_.push_back(ego_status_);
  }

  auto createEntity(const std::string & name, const uint8_t type, const uint8_t subtype) -> void
  {
    EntityStatus status;
    initializeEntity(status, name, type, subtype, entity_pose_, entity_dimensions_);
    status_.push_back(status);
    lidar_detected_entities_.push_back(name);
  }

private:
  auto configureDetectionSensor() -> void
  {
    config_.set_entity("ego");
    config_.set_architecture_type("awf/universe");
    config_.set_update_duration(0.1);
    config_.set_object_recognition_delay(0.0);
    config_.set_object_recognition_ground_truth_delay(0.0);
    config_.set_pos_noise_stddev(0.0);
    config_.set_probability_of_lost(0.0);
    config_.set_detect_all_objects_in_range(true);
    config_.set_range(100.0);
    config_.set_random_seed(12345);
  }

  auto createRosInterface() -> void
  {
    detected_objects_publisher_ =
      node_->create_publisher<DetectedObjectsMsg>("detected_objects_output", 10);
    ground_truth_objects_publisher_ =
      node_->create_publisher<TrackedObjectsMsg>("tracked_objects_output", 10);

    detected_objects_subscriber_ = node_->create_subscription<DetectedObjectsMsg>(
      "detected_objects_output", 10,
      [this](const DetectedObjectsMsg::SharedPtr msg) { received_msg_ = msg; });
  }

  auto initializeEntity(
    EntityStatus & status, const std::string & name, const uint8_t type, const uint8_t subtype,
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions) -> void
  {
    status.set_name(name);

    EntityType entity_type;
    entity_type.type = static_cast<traffic_simulator_msgs::EntityType_Enum>(type);
    EntitySubtype entity_subtype;
    entity_subtype.value = static_cast<traffic_simulator_msgs::EntitySubtype_Enum>(subtype);

    simulation_interface::toProto(entity_type, *status.mutable_type());
    simulation_interface::toProto(entity_subtype, *status.mutable_subtype());

    auto new_pose = status.mutable_pose();
    auto new_position = new_pose->mutable_position();
    new_position->set_x(pose.position.x);
    new_position->set_y(pose.position.y);
    new_position->set_z(pose.position.z);

    auto new_orientation = new_pose->mutable_orientation();
    new_orientation->set_x(pose.orientation.x);
    new_orientation->set_y(pose.orientation.y);
    new_orientation->set_z(pose.orientation.z);
    new_orientation->set_w(pose.orientation.w);

    auto new_bounding_box = status.mutable_bounding_box();
    auto new_dimensions = new_bounding_box->mutable_dimensions();
    new_dimensions->set_x(dimensions.x);
    new_dimensions->set_y(dimensions.y);
    new_dimensions->set_z(dimensions.z);
  }

  auto createPose(double px, double py, double pz, double ox, double oy, double oz, double ow)
    -> geometry_msgs::msg::Pose
  {
    return geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(px).y(py).z(pz))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(ox).y(oy).z(oz).w(ow));
  }

  auto createDimensions(double x, double y, double z) -> geometry_msgs::msg::Vector3
  {
    return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(y).z(z);
  }
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_DETECTION_SENSOR_HPP_
