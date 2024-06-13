
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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <simple_sensor_simulator/sensor_simulation/lidar/config/lidar_parameters.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/config/voxel_grid_occlusion_estimation_parameters.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/utils.hpp>
#include <string>
#include <utility>

namespace simple_sensor_simulator
{
Raycaster::Raycaster(const LidarParameters & lidar_parameters)
: lidar_parameters_(lidar_parameters),
  point_cloud_manager_(std::make_unique<PointCloudManager>(lidar_parameters)),
  combined_pointcloud_(std::make_shared<PointCloud>()),
  occlusion_estimator_(),
  sensor_origin_(0.0f, 0.0f, 0.0f, 1.0f),
  next_label_(1)
{
  const float leaf_size = VoxelGridOcclusionEstimationParameters::getLeafSize();
  occlusion_estimator_.setLeafSize(leaf_size, leaf_size, leaf_size);
  combined_pointcloud_->reserve(estimateCombinedPointCloudSize());
}

Raycaster::~Raycaster() {}

auto Raycaster::raycast(
  const std::string & frame_id, const rclcpp::Time & stamp,
  const geometry_msgs::msg::Pose & sensor_origin) -> const PointCloud2
{
  sensor_origin_ = Eigen::Vector4f(
    sensor_origin.position.x, sensor_origin.position.y, sensor_origin.position.z, 1.0f);

  detected_targets_.clear();
  combinePointClouds();

  auto visible_cloud = evaluateOcclusion();

  auto pointcloud_msg = utils::convertToPointCloud2(combined_pointcloud_);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;

  detection_targets_.clear();
  next_label_ = 1;

  return pointcloud_msg;
}

auto Raycaster::evaluateOcclusion() -> PointCloudPtr
{
  static constexpr int point_cloud_size_threshold =
    VoxelGridOcclusionEstimationParameters::getPointCloudSizeThreshold();

  auto output_cloud = std::make_shared<PointCloud>();
  output_cloud->reserve(combined_pointcloud_->size());
  std::unordered_set<int> detected_labels;

  if (combined_pointcloud_->size() < point_cloud_size_threshold) {
    evaluateOcclusionSequential(output_cloud, detected_labels);
  } else {
    evaluateOcclusionParallel(output_cloud, detected_labels);
  }

  detectTargets(detected_labels);
  return output_cloud;
}

auto Raycaster::evaluateOcclusionSequential(
  PointCloudPtr & output_cloud, std::unordered_set<int> & detected_labels) -> void
{
  occlusion_estimator_.setInputCloud(combined_pointcloud_);
  occlusion_estimator_.setSensorOrigin(sensor_origin_);
  occlusion_estimator_.initializeVoxelGrid();

  for (const auto & point : *combined_pointcloud_) {
    if (isPointValid(point)) {
      int point_status;
      Eigen::Vector3i voxel_coord =
        occlusion_estimator_.getGridCoordinates(point.x, point.y, point.z);
      if (
        occlusion_estimator_.occlusionEstimation(point_status, voxel_coord) == 0 &&
        point_status == 0) {
        detected_labels.insert(point.label);
        output_cloud->push_back(point);
      }
    }
  }
}

auto Raycaster::evaluateOcclusionParallel(
  PointCloudPtr & output_cloud, std::unordered_set<int> & detected_labels) -> void
{
  static constexpr float leaf_size = VoxelGridOcclusionEstimationParameters::getLeafSize();

  PointType min_point, max_point;
  calculateBoundingBox(min_point, max_point);

  const float slice_height = (max_point.z - min_point.z) / std::thread::hardware_concurrency() / 2;
  const float overlap = 0.1f * slice_height;

  int max_threads = std::thread::hardware_concurrency() / 2;
  std::vector<std::unordered_set<int>> thread_local_detected_labels(max_threads);
  std::vector<std::future<PointCloudPtr>> futures(max_threads);

  for (int thread_index = 0; thread_index < max_threads; ++thread_index) {
    futures[thread_index] = std::async(
      std::launch::async, [this, &thread_local_detected_labels, thread_index, min_point, max_point,
                           slice_height, overlap, max_threads]() {
        PointCloudPtr local_output_cloud = std::make_shared<PointCloud>();
        local_output_cloud->reserve(combined_pointcloud_->size() / max_threads);

        OcclusionEstimator local_occlusion_estimator;
        local_occlusion_estimator.setLeafSize(leaf_size, leaf_size, leaf_size);
        local_occlusion_estimator.setInputCloud(combined_pointcloud_);
        local_occlusion_estimator.setSensorOrigin(sensor_origin_);
        local_occlusion_estimator.initializeVoxelGrid();

        float start_z = min_point.z + thread_index * slice_height;
        float end_z = start_z + slice_height;
        float extended_start_z = std::max(min_point.z, start_z - overlap);
        float extended_end_z = std::min(max_point.z, end_z + overlap);

        for (const auto & point : *combined_pointcloud_) {
          if (
            isPointInExtendedRange(point, extended_start_z, extended_end_z) &&
            isPointValid(point)) {
            int point_status;
            Eigen::Vector3i voxel_coord =
              local_occlusion_estimator.getGridCoordinates(point.x, point.y, point.z);
            if (
              local_occlusion_estimator.occlusionEstimation(point_status, voxel_coord) == 0 &&
              point_status == 0) {
              thread_local_detected_labels[thread_index].insert(point.label);
              local_output_cloud->push_back(point);
            }
          }
        }

        PointCloudPtr core_output_cloud = std::make_shared<PointCloud>();
        core_output_cloud->reserve(local_output_cloud->size());
        for (const auto & point : *local_output_cloud) {
          if (isPointInCoreRange(point, start_z, end_z)) {
            core_output_cloud->push_back(point);
          }
        }

        return core_output_cloud;
      });
  }

  for (auto & future : futures) {
    if (future.valid()) {
      auto local_output_cloud = future.get();
      if (local_output_cloud) {
        *output_cloud += *local_output_cloud;
      }
    }
  }

  for (const auto & local_labels : thread_local_detected_labels) {
    detected_labels.insert(local_labels.begin(), local_labels.end());
  }
}

auto Raycaster::calculateBoundingBox(PointType & min_point, PointType & max_point) -> void
{
  static constexpr int point_cloud_size_threshold =
    VoxelGridOcclusionEstimationParameters::getPointCloudSizeThreshold();

  min_point.x = min_point.y = min_point.z = std::numeric_limits<float>::max();
  max_point.x = max_point.y = max_point.z = -std::numeric_limits<float>::max();

  if (combined_pointcloud_->size() < point_cloud_size_threshold) {
    // Sequential processing for bounding box calculation
    for (const auto & point : *combined_pointcloud_) {
      updateBoundingBox(point, min_point, max_point);
    }
  } else {
    // Parallel processing for bounding box calculation
    int max_threads = std::thread::hardware_concurrency() / 2;
    std::vector<std::future<void>> futures(max_threads);
    std::mutex min_max_mutex;

    for (int thread_index = 0; thread_index < max_threads; ++thread_index) {
      futures[thread_index] = std::async(
        std::launch::async,
        [this, &min_point, &max_point, &min_max_mutex, thread_index, max_threads]() {
          PointType local_min, local_max;
          local_min.x = local_min.y = local_min.z = std::numeric_limits<float>::max();
          local_max.x = local_max.y = local_max.z = -std::numeric_limits<float>::max();

          size_t start_index = thread_index * combined_pointcloud_->size() / max_threads;
          size_t end_index = (thread_index == max_threads - 1)
                               ? combined_pointcloud_->size()
                               : start_index + combined_pointcloud_->size() / max_threads;

          for (size_t i = start_index; i < end_index; ++i) {
            updateBoundingBox(combined_pointcloud_->points[i], local_min, local_max);
          }

          {
            std::lock_guard<std::mutex> lock(min_max_mutex);
            updateBoundingBox(local_min, min_point, max_point);
            updateBoundingBox(local_max, min_point, max_point);
          }
        });
    }

    for (auto & future : futures) {
      if (future.valid()) {
        future.get();
      }
    }
  }
}

auto Raycaster::updateBoundingBox(
  const PointType & point, PointType & min_point, PointType & max_point) -> void
{
  if (point.x < min_point.x) min_point.x = point.x;
  if (point.y < min_point.y) min_point.y = point.y;
  if (point.z < min_point.z) min_point.z = point.z;

  if (point.x > max_point.x) max_point.x = point.x;
  if (point.y > max_point.y) max_point.y = point.y;
  if (point.z > max_point.z) max_point.z = point.z;
}

auto Raycaster::isPointInExtendedRange(
  const PointType & point, float extended_start_z, float extended_end_z) const -> bool
{
  return (point.z >= extended_start_z && point.z < extended_end_z);
}

auto Raycaster::isPointInCoreRange(const PointType & point, float start_z, float end_z) const
  -> bool
{
  return (point.z >= start_z && point.z < end_z);
}

auto Raycaster::combinePointClouds() -> void
{
  static constexpr int point_cloud_size_threshold =
    VoxelGridOcclusionEstimationParameters::getPointCloudSizeThreshold();

  int max_threads = std::thread::hardware_concurrency() / 2;
  combined_pointcloud_->clear();

  const auto max_sensor_range = lidar_parameters_.getMaxRange();
  for (const auto & target : detection_targets_) {
    float distance_to_sensor = utils::calculateDistance(target.second->getPose(), sensor_origin_);
    target.second->updateCurrentPointCloud(distance_to_sensor, max_sensor_range);
  }

  std::mutex local_mutex;
  std::atomic<int> active_threads(0);
  std::vector<std::future<void>> futures;

  for (const auto & target_pair : detection_targets_) {
    auto point_cloud = target_pair.second->getPointCloud();
    if (point_cloud) {
      if (
        point_cloud->points.size() < point_cloud_size_threshold || active_threads >= max_threads) {
        Eigen::Affine3f transform = utils::createTransform(target_pair.second->getPose());
        pcl::PointCloud<pcl::PointXYZL>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZL>);
        utils::transformPointCloud(point_cloud, transformed_cloud, transform);
        std::lock_guard<std::mutex> lock(local_mutex);
        *combined_pointcloud_ += *transformed_cloud;
      } else {
        active_threads++;
        futures.emplace_back(std::async(
          std::launch::async,
          [this, point_cloud, target = target_pair.second, &local_mutex, &active_threads]() {
            Eigen::Affine3f transform = utils::createTransform(target->getPose());
            pcl::PointCloud<pcl::PointXYZL>::Ptr transformed_cloud(
              new pcl::PointCloud<pcl::PointXYZL>);
            utils::transformPointCloud(point_cloud, transformed_cloud, transform);
            std::lock_guard<std::mutex> lock(local_mutex);
            *combined_pointcloud_ += *transformed_cloud;
            active_threads--;
          }));
      }
    }
  }

  for (auto & future : futures) {
    if (future.valid()) {
      future.get();
    }
  }
}

auto Raycaster::isPointValid(const PointType & point) -> bool
{
  return isPointInSensorRange(point) && isPointInVerticalFOV(point) &&
         isPointInHorizontalFOV(point);
}

auto Raycaster::isPointInSensorRange(const PointType & point) const -> bool
{
  float distance = utils::calculateDistance(point, sensor_origin_);
  return distance <= lidar_parameters_.getMaxRange() && distance > lidar_parameters_.getMinRange();
}

auto Raycaster::isPointInVerticalFOV(const PointType & point) const -> bool
{
  const float vertical_fov = lidar_parameters_.getVerticalFOV();
  float corrected_z = point.z - sensor_origin_[2];
  float elevation_angle =
    utils::radToDeg(std::atan2(corrected_z, std::sqrt(point.x * point.x + point.y * point.y)));
  return elevation_angle >= -vertical_fov && elevation_angle <= vertical_fov;
}

auto Raycaster::isPointInHorizontalFOV(const PointType & point) const -> bool
{
  float azimuth_angle = utils::radToDeg(std::atan2(point.y, point.x));
  azimuth_angle = std::fmod(azimuth_angle + 360.0, 360.0);
  return azimuth_angle >= 0 && azimuth_angle <= lidar_parameters_.getHorizontalFOV();
}

auto Raycaster::detectTargets(const std::unordered_set<int> & detected_labels) -> void
{
  detected_targets_.clear();
  for (const auto & target : detection_targets_) {
    if (detected_labels.find(target.second->getLabel()) != detected_labels.end()) {
      detected_targets_.push_back(target.first);
    }
  }
}

auto Raycaster::estimateCombinedPointCloudSize() const -> size_t
{
  size_t total_points = 0;
  for (const auto & target : detection_targets_) {
    total_points += target.second->getPointCloudSize();
  }
  return total_points;
}
}  // namespace simple_sensor_simulator