// Copyright 2015-2022 Tier IV, Inc. All rights reserved.
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.


#include "scenario_generator/scenario_generator.hpp"


ScenarioGenerator::ScenarioGenerator(const rclcpp::NodeOptions & option)
: Node("scenario_generator", option)
{
  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "trajectory/initial_pose", 1, std::bind(&ScenarioGenerator::onInitPose, this, std::placeholders::_1));

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "trajectory/goal_pose", 1, std::bind(&ScenarioGenerator::onGoalPose, this, std::placeholders::_1));
}

void ScenarioGenerator::onInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  init_pose_ = msg.get()->pose.pose;

  RCLCPP_INFO_STREAM(get_logger(), "Received new init pose: (x: " << init_pose_.position.x << ", y:"
                                                                  << init_pose_.position.y << ", z: "
                                                                  << init_pose_.position.z << ")");
}

void ScenarioGenerator::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received new goal pose: (x: " << msg->pose.position.x << ", y:"
                                                                  << msg->pose.position.y << ", z: "
                                                                  << msg->pose.position.z << ")");
}
