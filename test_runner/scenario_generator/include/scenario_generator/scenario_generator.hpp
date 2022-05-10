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

#ifndef SCENARIO_GENERATOR__SCENARIO_GENERATOR_HPP
#define SCENARIO_GENERATOR__SCENARIO_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include "traffic_simulator/hdmap_utils/hdmap_utils.hpp"
#include "traffic_simulator/behavior/route_planner.hpp"


enum class TrajectoryOption
{
  NONE,
  VEHICLE,
  PEDESTRIAN
};

class ScenarioGenerator : public rclcpp::Node
{
public:
  explicit ScenarioGenerator(const rclcpp::NodeOptions & option);

private:
  void onInitPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void onGoalPose(geometry_msgs::msg::PoseStamped::SharedPtr);
  void onOption(std_msgs::msg::String::SharedPtr);

  void handleVehicleInitPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void handlePedestrianInitPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
  void handleVehicleGoalPose(geometry_msgs::msg::PoseStamped::SharedPtr);
  void handlePedestrianGoalPose(geometry_msgs::msg::PoseStamped::SharedPtr);

  std::vector<geometry_msgs::msg::Pose> transformToUnityFrame(const std::vector<geometry_msgs::msg::Pose> &);

  void publishVisualization(const std::vector<geometry_msgs::msg::Pose> &);
  void generatePythonCode(const std::vector<geometry_msgs::msg::Pose> &trajectory);

  void clear();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_type_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;

  TrajectoryOption current_trajectory_option_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::RoutePlanner> route_planner_ptr_;

  std::vector<geometry_msgs::msg::Pose> collected_poses_;

  geometry_msgs::msg::Vector3 lanelet_to_unity_tf_;

  int trajectory_print_counter_{0};
};

#endif  // SCENARIO_GENERATOR__SCENARIO_GENERATOR_HPP
