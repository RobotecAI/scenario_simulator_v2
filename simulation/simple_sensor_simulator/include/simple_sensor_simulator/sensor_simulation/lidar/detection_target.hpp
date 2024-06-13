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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__DETECTION_TARGET_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__DETECTION_TARGET_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{

// Forward declarations
class PointCloudManager;

using PointType = pcl::PointXYZL;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointCloud = pcl::PointCloud<PointType>;

class DetectionTarget
{
public:
  using TargetName = std::string;

  DetectionTarget(
    const TargetName & name, const Eigen::Vector3f & dimensions,
    const geometry_msgs::msg::Pose & pose, int label, PointCloudManager & point_cloud_manager);

  auto getName() const -> TargetName { return name_; }
  auto getLabel() const -> int { return label_; }
  auto getPose() const -> const geometry_msgs::msg::Pose { return pose_; }
  auto getPointCloudSize() const -> int;
  auto getPointCloud() const -> PointCloudPtr;
  auto updateCurrentPointCloud(float distance_to_sensor, float max_sensor_range) -> void;

private:
  PointCloudPtr current_point_cloud_;
  const Eigen::Vector3f dimensions_;
  const geometry_msgs::msg::Pose pose_;
  const TargetName name_;
  const int label_;
  int previous_index_{-1};

  mutable std::mutex mutex_;

  PointCloudManager & point_cloud_manager_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__DETECTION_TARGET_HPP_