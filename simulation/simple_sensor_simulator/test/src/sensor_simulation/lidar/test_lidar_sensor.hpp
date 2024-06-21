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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <vector>

#include "../../utils/helper_functions.hpp"

using namespace simple_sensor_simulator;

using EntityType = traffic_simulator_msgs::EntityType;
using EntityStatus = traffic_simulator_msgs::EntityStatus;

class LidarSensorTest : public ::testing::Test
{
protected:
  LidarSensorTest() : config_(utils::constructLidarConfiguration("ego", "awf/universe", 0.0, 0.5))
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("lidar_sensor_test_node");
    createRosInterface();
    initializeEntityStatuses();

    lidar_ = std::make_unique<LidarSensor<sensor_msgs::msg::PointCloud2>>(0.0, config_, publisher_);
  }

  ~LidarSensorTest() { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  EntityStatus ego_status_;
  EntityStatus other1_status_;
  EntityStatus other2_status_;
  std::vector<EntityStatus> status_;

  std::unique_ptr<LidarSensorBase> lidar_;
  simulation_api_schema::LidarConfiguration config_;
  sensor_msgs::msg::PointCloud2::SharedPtr received_msg_;

  double current_simulation_time_{1.0};
  rclcpp::Time current_ros_time_{1};

private:
  auto initializeEntityStatuses() -> void
  {
    auto dimensions = utils::makeDimensions(4.5, 2.0, 1.5);

    ego_status_ = makeEntity(
      "ego", EntityType::EGO, utils::makePose(5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0), dimensions);
    other1_status_ = makeEntity(
      "other1", EntityType::VEHICLE, utils::makePose(-3.0, -3.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);
    other2_status_ = makeEntity(
      "other2", EntityType::VEHICLE, utils::makePose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);

    status_ = {ego_status_, other1_status_, other2_status_};
  }

  auto makeEntity(
    const std::string & name, const EntityType::Enum type, const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Vector3 & dimensions) -> EntityStatus
  {
    EntityStatus status;

    status.set_name(name);
    status.mutable_type()->set_type(type);

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

    return status;
  }

  auto createRosInterface() -> void
  {
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_output", 10);
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar_output", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { received_msg_ = msg; });
  }
};
#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
