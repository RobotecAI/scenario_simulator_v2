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
#include <simple_sensor_simulator/sensor_simulation/lidar/config/lidar_parameters.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/config/voxel_grid_occlusion_estimation_parameters.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/point_cloud_manager.hpp>

namespace simple_sensor_simulator
{

PointCloudManager::PointCloudManager(const LidarParameters & lidar_parameters)
: lidar_parameters_(lidar_parameters)
{
  point_clouds_.reserve(10);
}

auto PointCloudManager::getPointCloud(const TargetName & target_name, int cloud_index) const
  -> PointCloudPtr
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = point_clouds_.find(target_name);
  if (it != point_clouds_.end() && cloud_index < static_cast<int>(it->second.first.size())) {
    return it->second.first[cloud_index];
  }
  return nullptr;
}

auto PointCloudManager::updatePointCloud(
  const TargetName & target_name, const Eigen::Vector3f & dimensions, const int label) -> void
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (isCloudRegenerationNeeded(target_name)) {
    generatePointClouds(target_name, dimensions, label);
  }
}

auto PointCloudManager::isCloudRegenerationNeeded(const TargetName & target_name) const -> bool
{
  static constexpr int num_clouds_per_target =
    VoxelGridOcclusionEstimationParameters::getNumCloudsPerTarget();
  auto it = point_clouds_.find(target_name);
  if (it == point_clouds_.end()) {
    return true;
  }
  const auto & meta = it->second.second;
  return meta.getNumCloudsPerTarget() != num_clouds_per_target ||
         meta.getSensorRange() != lidar_parameters_.getMaxRange();
}

auto PointCloudManager::generatePointClouds(
  const TargetName & target_name, const Eigen::Vector3f & dimensions, const int label) -> void
{
  static constexpr int num_clouds_per_target =
    VoxelGridOcclusionEstimationParameters::getNumCloudsPerTarget();

  point_clouds_[target_name].first.clear();

  float range_step = lidar_parameters_.getMaxRange() / num_clouds_per_target;
  for (int i = 0; i < num_clouds_per_target; ++i) {
    float distance = range_step * (i + 1);
    float resolution = computePointCloudResolution(distance);
    createPointCloud(target_name, dimensions, resolution, label);
  }

  PointCloudMetaData new_meta;
  new_meta.setNumCloudsPerTarget(num_clouds_per_target);
  new_meta.setSensorRange(lidar_parameters_.getMaxRange());

  point_clouds_[target_name].second = new_meta;
}

auto PointCloudManager::computePointCloudResolution(float distance_to_sensor) const -> float
{
  static constexpr float slope = VoxelGridOcclusionEstimationParameters::getResolutionSlope();
  static constexpr float intercept =
    VoxelGridOcclusionEstimationParameters::getResolutionIntercept();
  constexpr static float min_resolution = 0.01f;

  float resolution = slope * distance_to_sensor - intercept;
  return std::max(resolution, min_resolution);
}

auto PointCloudManager::createPointCloud(
  const std::string & target_name, const Eigen::Vector3f & dimensions, float resolution,
  const int label) -> void
{
  PointCloudPtr cloud = std::make_shared<PointCloud>();

  float volume = dimensions.x() * dimensions.y() * dimensions.z();
  float point_density = 1.0f / (resolution * resolution * resolution);
  size_t estimated_num_points = static_cast<size_t>(volume * point_density);

  cloud->points.reserve(estimated_num_points);

  float half_depth = dimensions.x() / 2.0f;
  float half_width = dimensions.y() / 2.0f;
  float half_height = dimensions.z() / 2.0f;

  for (float y = -half_width; y <= half_width; y += resolution) {
    for (float z = -half_height; z <= half_height; z += resolution) {
      PointType point;
      point.label = label;

      point.x = half_depth;
      point.y = y;
      point.z = z;
      cloud->points.emplace_back(point);

      point.x = -half_depth;
      cloud->points.emplace_back(point);
    }
  }

  for (float x = -half_depth; x <= half_depth; x += resolution) {
    for (float z = -half_height; z <= half_height; z += resolution) {
      PointType point;
      point.label = label;

      point.x = x;
      point.y = -half_width;
      point.z = z;
      cloud->points.emplace_back(point);

      point.y = half_width;
      cloud->points.emplace_back(point);
    }
  }

  for (float x = -half_depth; x <= half_depth; x += resolution) {
    for (float y = -half_width; y <= half_width; y += resolution) {
      PointType point;
      point.label = label;

      point.x = x;
      point.y = y;
      point.z = half_height;
      cloud->points.emplace_back(point);

      point.z = -half_height;
      cloud->points.emplace_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  point_clouds_[target_name].first.emplace_back(cloud);
}

}  // namespace simple_sensor_simulator