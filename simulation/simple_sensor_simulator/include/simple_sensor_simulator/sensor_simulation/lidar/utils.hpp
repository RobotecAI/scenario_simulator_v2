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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__UTILS_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__UTILS_HPP_

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace simple_sensor_simulator
{

using PointType = pcl::PointXYZL;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;

namespace utils
{
constexpr auto degToRad(float degrees) -> float { return degrees * (M_PI / 180.0f); }

constexpr auto radToDeg(float radians) -> float { return radians * (180.0f / M_PI); }

inline auto convertToPointCloud2(const PointCloudPtr & pcl_cloud) -> PointCloud2
{
  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(*pcl_cloud, ros_cloud);
  return ros_cloud;
}

inline auto createTransform(const geometry_msgs::msg::Pose & pose) -> Eigen::Affine3f
{
  Eigen::Translation3f translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaternionf rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Affine3f transform = translation * rotation;
  return transform;
}

inline auto transformPointCloud(
  const PointCloudPtr & input_cloud, PointCloudPtr & output_cloud,
  const Eigen::Affine3f & transform) -> void
{
  pcl::transformPointCloud(*input_cloud, *output_cloud, transform);
}

inline auto calculateDistance(const PointType & point1, const Eigen::Vector4f & point2) -> float
{
  return (point1.getVector4fMap() - point2).norm();
}

inline auto calculateDistance(
  const geometry_msgs::msg::Pose & point1, const Eigen::Vector4f & point2) -> float
{
  Eigen::Vector3f position(point1.position.x, point1.position.y, point1.position.z);
  return (position - point2.head<3>()).norm();
}
}  // namespace utils
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__UTILS_HPP_