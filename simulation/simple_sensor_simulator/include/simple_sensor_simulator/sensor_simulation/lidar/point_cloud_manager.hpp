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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__POINT_CLOUD_MANAGER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__POINT_CLOUD_MANAGER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{

// Forward declarations
class LidarParameters;

using PointType = pcl::PointXYZL;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointCloud = pcl::PointCloud<PointType>;

struct PointCloudMetaData
{
  int num_clouds_per_target_;
  float sensor_range_;

  auto getSensorRange() const -> float { return sensor_range_; }
  auto getNumCloudsPerTarget() const -> int { return num_clouds_per_target_; }
  auto setSensorRange(float new_sensor_range) { sensor_range_ = new_sensor_range; }
  auto setNumCloudsPerTarget(int new_num_clouds) { num_clouds_per_target_ = new_num_clouds; }
};

class PointCloudManager
{
public:
  using TargetName = std::string;
  using PointCloudData = std::pair<std::vector<PointCloudPtr>, PointCloudMetaData>;

  explicit PointCloudManager(const LidarParameters & lidar_parameters);

  auto getPointCloud(const TargetName & target_name, int cloud_index) const -> PointCloudPtr;
  auto updatePointCloud(
    const TargetName & name, const Eigen::Vector3f & dimensions, const int label) -> void;

private:
  std::unordered_map<TargetName, PointCloudData> point_clouds_;
  const LidarParameters & lidar_parameters_;
  mutable std::mutex mutex_;

  auto isCloudRegenerationNeeded(const TargetName & target_name) const -> bool;
  auto generatePointClouds(
    const TargetName & target_name, const Eigen::Vector3f & dimensions, const int label) -> void;
  auto computePointCloudResolution(float distance_to_sensor) const -> float;
  auto createPointCloud(
    const TargetName & target_name, const Eigen::Vector3f & dimensions, float resolution,
    const int label) -> void;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__POINT_CLOUD_MANAGER_HPP_