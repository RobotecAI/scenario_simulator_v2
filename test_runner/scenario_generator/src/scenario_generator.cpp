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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "traffic_simulator/api/configuration.hpp"


const double PLANNING_HORIZON_M = 300.0;


double getYawDeg(const geometry_msgs::msg::Quaternion & q_msg)
{
  double roll, pitch, yaw;
  auto q = tf2::Quaternion(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  auto m = tf2::Matrix3x3(q);
  m.getRPY(roll, pitch, yaw);
//  std::cout << "roll: " << roll * 180.0 / M_PI << ", pitch: " << pitch * 180.0 / M_PI << ", yaw: " << yaw * 180.0 / M_PI << std::endl;

  auto yaw_deg = yaw * 180.0 / M_PI;

  // unity transformation: (workaround) easier to do on angles than on quaternions
  auto yaw_unity = -yaw_deg - 90;

  return yaw_unity;
}

ScenarioGenerator::ScenarioGenerator(const rclcpp::NodeOptions & option)
: Node("scenario_generator", option)
{
  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "trajectory/initial_pose", 1, std::bind(&ScenarioGenerator::onInitPose, this, std::placeholders::_1));
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "trajectory/goal_pose", 1, std::bind(&ScenarioGenerator::onGoalPose, this, std::placeholders::_1));
  trajectory_type_sub_ = this->create_subscription<std_msgs::msg::String>(
    "trajectory/option", 1, std::bind(&ScenarioGenerator::onOption, this, std::placeholders::_1));

  planned_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory/planned_path", 1);

  std::string map_path = this->declare_parameter<std::string>("lanelet_map_path", "");
  hdmap_utils_ptr_ =
      std::make_shared<hdmap_utils::HdMapUtils>(map_path, geographic_msgs::msg::GeoPoint());

  route_planner_ptr_ = std::make_shared<traffic_simulator::RoutePlanner>(hdmap_utils_ptr_);

  // based on Shinjuku scene settings in Unity
  lanelet_to_unity_tf_.x = 81655.73;
  lanelet_to_unity_tf_.y = 50137.43;
  lanelet_to_unity_tf_.z = 42.49998;
}

void ScenarioGenerator::onOption(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "p")
  {
    std::cout << "Changed mode to pedestrian." << std::endl;
    current_trajectory_option_ = TrajectoryOption::PEDESTRIAN;
  }
  else if (msg->data == "v")
  {
    std::cout << "Changed mode to vehicle" << std::endl;
    current_trajectory_option_ = TrajectoryOption::VEHICLE;
  }
  else
  {
    std::cout << "Invalid option " << std::quoted(msg->data) << std::endl;
  }
}

void ScenarioGenerator::onInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Received new init pose: (x: " << msg->pose.pose.position.x
                                                       << ", y:" << msg->pose.pose.position.y
                                                       << ", z: " << msg->pose.pose.position.z << ")");

  switch (current_trajectory_option_) {
    case TrajectoryOption::NONE:
      std::cout << "Choose trajectory generation option first by publishing on 'trajectory/option' topic." << std::endl;
      return;
    case TrajectoryOption::PEDESTRIAN:
      handlePedestrianInitPose(msg);
      break;
    case TrajectoryOption::VEHICLE:
      handleVehicleInitPose(msg);
      break;
  }

  publishVisualization(collected_poses_);
}

void ScenarioGenerator::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (collected_poses_.empty())
  {
    std::cout << "Set the init pose before setting the goal pose." << std::endl;
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Received new goal pose: (x: " << msg->pose.position.x
                                                       << ", y:" << msg->pose.position.y
                                                       << ", z: " << msg->pose.position.z << ")");

  switch (current_trajectory_option_) {
    case TrajectoryOption::NONE:
      std::cout << "Choose trajectory generation option first by publishing on 'trajectory/option' topic." << std::endl;
      return;
    case TrajectoryOption::PEDESTRIAN:
      handlePedestrianGoalPose(msg);
      break;
    case TrajectoryOption::VEHICLE:
      handleVehicleGoalPose(msg);
      break;
  }

  publishVisualization(collected_poses_);
  auto transformed_trajectory = transformToUnityFrame(collected_poses_);
  printPythonCode(transformed_trajectory);
  clear();
}

void ScenarioGenerator::handleVehicleInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  collected_poses_.clear();
  collected_poses_.push_back(msg.get()->pose.pose);
}

void ScenarioGenerator::handlePedestrianInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // do not clear init pose
  // pedestrian's trajectory is just a collection of N init poses and 1 goal psoe
  collected_poses_.push_back(msg.get()->pose.pose);
}

void ScenarioGenerator::handleVehicleGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  auto initLaneletPose = hdmap_utils_ptr_->toLaneletPose(collected_poses_[0], true);
  auto goalLaneletPose = hdmap_utils_ptr_->toLaneletPose(msg->pose, true);

  if (!initLaneletPose)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Init pose is not on the lanelet!");
    return;
  }

  if (!goalLaneletPose)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Goal pose is not on the lanelet!");
    return;
  }

  auto route_lanelets = route_planner_ptr_->getRouteLanelets(initLaneletPose.get(), goalLaneletPose.get(),
                                                             PLANNING_HORIZON_M);
  auto spline = std::make_shared<traffic_simulator::math::CatmullRomSpline>(
      hdmap_utils_ptr_->getCenterPoints(route_lanelets));
  auto goalS = spline->getSValue(msg->pose);
  collected_poses_ = spline->getOrientedTrajectory(initLaneletPose.get().s, goalS.get(), 1);
}

void ScenarioGenerator::handlePedestrianGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  collected_poses_.push_back(msg.get()->pose);
}

void ScenarioGenerator::clear()
{
  // cleanup after the trajectory is finished
  collected_poses_.clear();
}

std::vector<geometry_msgs::msg::Pose>
ScenarioGenerator::transformToUnityFrame(const std::vector<geometry_msgs::msg::Pose> &trajectory)
{
  std::vector<geometry_msgs::msg::Pose> transformed_trajectory = trajectory;

  for (auto & pose : transformed_trajectory)
  {
    // apply mgrs offset
    pose.position.x -= lanelet_to_unity_tf_.x;
    pose.position.y -= lanelet_to_unity_tf_.y;
    pose.position.z -= lanelet_to_unity_tf_.z;

    // change to Unity coordinates
    auto pose_copy = pose;
    pose.position.x = -pose_copy.position.x;
    pose.position.y = pose_copy.position.z;
    pose.position.z = -pose_copy.position.y;

//    std::cout << "x: " << pose.orientation.x << ", y: " << pose.orientation.y << ", z: " << pose.orientation.z << ", w: " << pose.orientation.w << std::endl;
//    pose.orientation.z = pose_copy.orientation.z;
//    pose.orientation.w = -pose_copy.orientation.w;
  }

  return transformed_trajectory;
}

void ScenarioGenerator::publishVisualization(const std::vector<geometry_msgs::msg::Pose> & trajectory)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = get_clock()->now();

  nav_msgs::msg::Path path;
  path.header = header;

  for (const auto & pose : trajectory)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header = header;
    path.poses.push_back(pose_stamped);
  }

  planned_path_pub_->publish(path);
}

void ScenarioGenerator::printPythonCode(const std::vector<geometry_msgs::msg::Pose> & trajectory)
{
  std::string trajectory_name = "tr_" + std::to_string(trajectory_print_counter_);
  std::cout << "\nThis is your python code:\n```" << std::endl;
  std::cout << trajectory_name << " = Trajectory(default_speed=2.5)" << std::endl;  // TODO: parametrize speed
  for (const auto & pose : trajectory)
  {
    std::cout << trajectory_name << ".move_abs(pose=Pose(x=" << pose.position.x <<
                                                      ", y=" << pose.position.y <<
                                                      ", z=" << pose.position.z <<
                                                      ", yaw=" << getYawDeg(pose.orientation) <<
                                                      "))" << std::endl;
  }
  std::cout << "```\n" << std::endl;

  trajectory_print_counter_++;
}
