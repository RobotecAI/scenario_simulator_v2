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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/detection_target.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/enhanced_voxel_grid_occlusion_estimation.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/point_cloud_manager.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace simple_sensor_simulator
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;

// Forward declarations
class LidarParameters;

class Raycaster
{
public:
  using TargetName = std::string;
  using OcclusionEstimator = pcl::EnhancedVoxelGridOcclusionEstimation<PointType>;

  explicit Raycaster(const LidarParameters & lidar_parameters_);
  ~Raycaster();

  template <typename T, typename... Ts>
  auto addDetectionTarget(TargetName name, Ts &&... xs) -> void
  {
    if (detection_targets_.count(name) != 0) {
      throw std::runtime_error("DetectionTarget " + name + " already exist.");
    }
    int label = next_label_++;
    auto detection_target =
      std::make_shared<T>(name, std::forward<Ts>(xs)..., label, *(point_cloud_manager_.get()));
    detection_targets_.emplace(name, detection_target);
  }

  auto raycast(
    const std::string & frame_id, const rclcpp::Time & stamp,
    const geometry_msgs::msg::Pose & sensor_origin) -> const PointCloud2;

  auto getDetectedTargets() const -> const std::vector<TargetName> { return detected_targets_; }

private:
  std::unique_ptr<PointCloudManager> point_cloud_manager_;
  PointCloudPtr combined_pointcloud_;
  OcclusionEstimator occlusion_estimator_;
  const LidarParameters & lidar_parameters_;
  Eigen::Vector4f sensor_origin_;
  int next_label_;
  std::unordered_map<TargetName, std::shared_ptr<DetectionTarget>> detection_targets_;
  std::vector<TargetName> detected_targets_;
  std::mutex combined_mutex_;

  auto evaluateOcclusion() -> PointCloudPtr;
  auto combinePointClouds() -> void;
  auto detectTargets(const std::unordered_set<int> & detected_labels) -> void;
  auto estimateCombinedPointCloudSize() const -> size_t;
  auto isPointValid(const PointType & point) -> bool;
  auto isPointInSensorRange(const PointType & point) const -> bool;
  auto isPointInVerticalFOV(const PointType & point) const -> bool;
  auto isPointInHorizontalFOV(const PointType & point) const -> bool;
  auto isPointInExtendedRange(
    const PointType & point, float extended_start_z, float extended_end_z) const -> bool;
  auto isPointInCoreRange(const PointType & point, float start_z, float end_z) const -> bool;
  auto calculateBoundingBox(PointType & min_point, PointType & max_point) -> void;
  auto updateBoundingBox(const PointType & point, PointType & min_point, PointType & max_point)
    -> void;
  auto evaluateOcclusionSequential(
    PointCloudPtr & output_cloud, std::unordered_set<int> & detected_labels) -> void;
  auto evaluateOcclusionParallel(
    PointCloudPtr & output_cloud, std::unordered_set<int> & detected_labels) -> void;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_