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

#include <simple_sensor_simulator/sensor_simulation/lidar/config/voxel_grid_occlusion_estimation_parameters.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/detection_target.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/point_cloud_manager.hpp>

namespace simple_sensor_simulator
{
DetectionTarget::DetectionTarget(
  const TargetName & name, const Eigen::Vector3f & dimensions,
  const geometry_msgs::msg::Pose & pose, int label, PointCloudManager & point_cloud_mgr)
: current_point_cloud_(nullptr),
  dimensions_(dimensions),
  pose_(pose),
  name_(name),
  label_(label),
  point_cloud_manager_(point_cloud_mgr)
{
  point_cloud_manager_.updatePointCloud(name_, dimensions_, label_);
  current_point_cloud_ = point_cloud_manager_.getPointCloud(name_, 0);
}

auto DetectionTarget::getPointCloudSize() const -> int
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_point_cloud_ ? current_point_cloud_->points.size() : 0;
}

auto DetectionTarget::getPointCloud() const -> PointCloudPtr
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_point_cloud_;
}

auto DetectionTarget::updateCurrentPointCloud(float distance_to_sensor, float max_sensor_range)
  -> void
{
  static constexpr int num_clouds_per_target =
    VoxelGridOcclusionEstimationParameters::getNumCloudsPerTarget();

  float range_step = max_sensor_range / num_clouds_per_target;
  int index = static_cast<int>(std::floor(distance_to_sensor / range_step));
  index = std::min(index, num_clouds_per_target - 1);

  if (index != previous_index_) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto point_cloud = point_cloud_manager_.getPointCloud(name_, index);
    if (point_cloud) {
      current_point_cloud_ = point_cloud;
      previous_index_ = index;
    }
  }
}

}  // namespace simple_sensor_simulator