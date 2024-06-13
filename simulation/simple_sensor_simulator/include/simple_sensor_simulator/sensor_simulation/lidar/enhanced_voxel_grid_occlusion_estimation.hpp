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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__ENHANCED_VOXEL_GRID_OCCLUSION_ESTIMATION_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__ENHANCED_VOXEL_GRID_OCCLUSION_ESTIMATION_HPP_

#include <pcl/filters/voxel_grid_occlusion_estimation.h>

namespace pcl
{
template <typename PointT>
class EnhancedVoxelGridOcclusionEstimation : public VoxelGridOcclusionEstimation<PointT>
{
public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  using Base = VoxelGridOcclusionEstimation<PointT>;
  using Base::Base;

  auto setSensorOrigin(const Eigen::Vector4f & origin) -> void { this->sensor_origin_ = origin; }
  auto getSensorOrigin() const -> Eigen::Vector4f { return this->sensor_origin_; }

  auto occlusionEstimation(int & out_state, const Eigen::Vector3i & in_target_voxel) -> int
  {
    if (!this->initialized_) {
      PCL_ERROR("Voxel grid not initialized; call initializeVoxelGrid() first! \n");
      return -1;
    }

    Eigen::Vector4f p = this->getCentroidCoordinate(in_target_voxel);
    Eigen::Vector4f direction = p - this->sensor_origin_;
    direction.normalize();

    float tmin = this->rayBoxIntersection(this->sensor_origin_, direction);

    if (tmin == -1) {
      out_state = 1;  // Assuming occluded if no intersection with detection target
      return -1;
    }

    out_state = this->rayTraversal(in_target_voxel, this->sensor_origin_, direction, tmin);
    return 0;
  }

protected:
  auto rayBoxIntersection(const Eigen::Vector4f & origin, const Eigen::Vector4f & direction)
    -> float
  {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    auto sign = [](float val) { return (val >= 0.0f) ? 1.0f : -1.0f; };

    tmin = (sign(direction[0]) * this->b_min_[0] - origin[0]) / direction[0];
    tmax = (sign(direction[0]) * this->b_max_[0] - origin[0]) / direction[0];

    tymin = (sign(direction[1]) * this->b_min_[1] - origin[1]) / direction[1];
    tymax = (sign(direction[1]) * this->b_max_[1] - origin[1]) / direction[1];

    if ((tmin > tymax) || (tymin > tmax)) return -1.0f;

    tmin = std::max(tmin, tymin);
    tmax = std::min(tmax, tymax);

    tzmin = (sign(direction[2]) * this->b_min_[2] - origin[2]) / direction[2];
    tzmax = (sign(direction[2]) * this->b_max_[2] - origin[2]) / direction[2];

    if ((tmin > tzmax) || (tzmin > tmax)) return -1.0f;

    tmin = std::max(tmin, tzmin);
    tmax = std::min(tmax, tzmax);

    return tmax >= 0.0f ? std::max(tmin, 0.0f) : -1.0f;
  }
};

}  // namespace pcl

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__ENHANCED_VOXEL_GRID_OCCLUSION_ESTIMATION_HPP_