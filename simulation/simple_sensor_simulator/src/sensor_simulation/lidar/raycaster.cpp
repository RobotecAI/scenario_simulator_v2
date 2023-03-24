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
#include <iostream>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <geometry/transform.hpp>
#include <rgl/utils/rgl_utils.hpp>

#define RAYCASTER_DEBUG 1
#if RAYCASTER_DEBUG == 1
#define CATCH_RGL_ERROR(err) { if (err != 0) {std::cout << "RGL error code: " << err << std::endl; breaker();} }
void breaker()   // put your breakpoint here
{
}
#else
#define CATCH_RGL_ERROR(err) err
#endif

namespace simple_sensor_simulator
{
Raycaster::Raycaster()
: primitive_ptrs_(0),
  device_(rtcNewDevice(nullptr)),
  scene_(rtcNewScene(device_)),
  engine_(seed_gen_())
{
}

Raycaster::Raycaster(std::string embree_config)
: primitive_ptrs_(0),
  device_(rtcNewDevice(embree_config.c_str())),
  scene_(rtcNewScene(device_)),
  engine_(seed_gen_())
{
}

void Raycaster::initRglNodes()
{
  use_rays_ = nullptr;
  raytrace_ = nullptr;
  compact_ = nullptr;

  assert( !rotation_matrices_rgl_.empty() );   // check whether ray directions have already been initialized
  CATCH_RGL_ERROR(rgl_node_rays_from_mat3x4f(&use_rays_, rotation_matrices_rgl_.data(), rotation_matrices_rgl_.size()));
  CATCH_RGL_ERROR(rgl_node_raytrace(&raytrace_, nullptr, 1));
  CATCH_RGL_ERROR(rgl_node_points_compact(&compact_));

  CATCH_RGL_ERROR(rgl_graph_node_add_child(use_rays_, raytrace_));
  CATCH_RGL_ERROR(rgl_graph_node_add_child(raytrace_, compact_));
}

Raycaster::~Raycaster()
{
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);
  CATCH_RGL_ERROR(rgl_graph_destroy(use_rays_));
}

void Raycaster::addEntity(const std::string & name, float depth, float width, float height)
{
  const auto it = entities_.find(name);   // if entity exists skip adding
  if (it != entities_.end()) {
    return;
  }
  rgl_mesh_t mesh = nullptr;
  rgl_vec3f vertices[8];
  rgl::initBoxVertices(vertices, depth, width, height);   // TODO check whether box is oriented in way it should
  rgl_vec3i indices[12];
  rgl::initBoxIndices(indices);
  CATCH_RGL_ERROR(rgl_mesh_create(&mesh, vertices, 8, indices, 12));   // Saves handle to a mesh in the pointer 'mesh'
  rgl_entity_t entity = nullptr;
  CATCH_RGL_ERROR(rgl_entity_create(&entity, nullptr, mesh));   // Saves handle to an entity in the pointer 'entity'
  entities_.insert({name, entity});   // Adds handle to entity to the map, every element in entity (including mesh referenced) are allocated by `rgl_` functions
  entities_pose_.insert({name, geometry_msgs::msg::Pose()});
}

bool Raycaster::setEntityPose(const std::string & name, const geometry_msgs::msg::Pose & pose)
{
  const auto it = entities_pose_.find(name);
  if (it == entities_pose_.end()) {
    return false;
  }
  it->second = pose;
  return true;
}

bool Raycaster::adjustPose(const std::string & name, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Pose & origin)
{
  const auto it = entities_.find(name);
  if (it == entities_.end()) {
    return false;
  }
  const auto pose_new = math::geometry::getRelativePose(origin, pose);
  rgl_mat3x4f entity_tf;
  rgl::setRglMatPose(entity_tf, pose_new, false);
  CATCH_RGL_ERROR(rgl_entity_set_pose(it->second, &entity_tf));
  return true;
}

bool Raycaster::setRglPoses(const geometry_msgs::msg::Pose & origin)
{
  for (const auto & [name, pose] : entities_pose_) {
    if (!adjustPose(name, pose, origin)) {
      return false;
    }
  }
  return true;
}

void Raycaster::setDirection(
  const simulation_api_schema::LidarConfiguration & configuration, double horizontal_angle_start,
  double horizontal_angle_end)
{
  std::vector<double> vertical_angles;
  for (const auto v : configuration.vertical_angles()) {
    vertical_angles.emplace_back(v);
  }

  auto quat_directions = getDirections(
    vertical_angles, horizontal_angle_start, horizontal_angle_end,
    configuration.horizontal_resolution());
  rotation_matrices_.clear();
  rotation_matrices_rgl_.clear();
  for (const auto & q : quat_directions) {
    rotation_matrices_.push_back(quaternion_operation::getRotationMatrix(q));
    rotation_matrices_rgl_.push_back(rgl::getRglMatRotation(q));   // Add ray rotations in RGL format
  }
  initRglNodes();   // init RGL here because ray directions are needed
}

std::vector<geometry_msgs::msg::Quaternion> Raycaster::getDirections(
  const std::vector<double> & vertical_angles, double horizontal_angle_start,
  double horizontal_angle_end, double horizontal_resolution)
{
  if (
    directions_.empty() || previous_horizontal_angle_start_ != horizontal_angle_start ||
    previous_horizontal_angle_end_ != horizontal_angle_end ||
    previous_horizontal_resolution_ != horizontal_resolution ||
    previous_vertical_angles_ != vertical_angles) {
    std::vector<geometry_msgs::msg::Quaternion> directions;
    double horizontal_angle = horizontal_angle_start;
    while (horizontal_angle <= horizontal_angle_end) {
      horizontal_angle = horizontal_angle + horizontal_resolution;
      for (const auto vertical_angle : vertical_angles) {
        geometry_msgs::msg::Vector3 rpy;
        rpy.x = 0;
        rpy.y = vertical_angle;
        rpy.z = horizontal_angle;
        auto quat = quaternion_operation::convertEulerAngleToQuaternion(rpy);
        directions.emplace_back(quat);
      }
    }
    directions_ = directions;
    previous_horizontal_angle_end_ = horizontal_angle_end;
    previous_horizontal_angle_start_ = horizontal_angle_start;
    previous_horizontal_resolution_ = horizontal_resolution;
    previous_vertical_angles_ = vertical_angles;
  }
  return directions_;
}

const std::vector<std::string> & Raycaster::getDetectedObject() const { return detected_objects_; }

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,   // origin is ego pose, no offset
  double max_distance, double min_distance)   // RGL does not support min_distance
{
  detected_objects_ = {};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto & pair : primitive_ptrs_) {
    auto id = pair.second->addToScene(device_, scene_);
    geometry_ids_.insert({id, pair.first});
  }
  setRglPoses(origin);
  CATCH_RGL_ERROR(rgl_node_raytrace(&raytrace_, nullptr, static_cast<float>(max_distance)));
  CATCH_RGL_ERROR(rgl_graph_run(use_rays_));
  int32_t out_count, out_size_of;
  CATCH_RGL_ERROR(rgl_graph_get_result_size(compact_, RGL_FIELD_XYZ_F32, &out_count, &out_size_of));
  std::vector<rgl_vec3f> results;
  results.resize(static_cast<size_t>(out_count));
  CATCH_RGL_ERROR(rgl_graph_get_result_data(compact_, RGL_FIELD_XYZ_F32, results.data()));
  for (const auto & r : results) {
    pcl::PointXYZI p;
    {
      p.x = r.value[0];
      p.y = r.value[1];
      p.z = r.value[2];
    }
    cloud->emplace_back(p);
    // TODO should add ID
  }

#if __DO_NOT_COMPILE_THIS__  // this legacy code is here only for reference
  // Run as many threads as physical cores (which is usually /2 virtual threads)
  // In heavy loads virtual threads (hyper-threading) add little to the overall performance
  // This also minimizes cost of creating a thread (roughly 10us on Intel/Linux)
  int thread_count = std::thread::hardware_concurrency() / 2;
  // Per thread data structures:
  std::vector<std::thread> threads(thread_count);
  std::vector<std::set<unsigned int>> thread_detected_ids(thread_count);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> thread_cloud(thread_count);

  rtcCommitScene(scene_);
  RTCIntersectContext context;
  for (unsigned int i = 0; i < threads.size(); ++i) {
    thread_cloud[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    threads[i] = std::thread(
      intersect, i, thread_count, scene_, thread_cloud[i], context, origin,
      std::ref(thread_detected_ids[i]), max_distance, min_distance, std::ref(rotation_matrices_));
  }
  for (unsigned int i = 0; i < threads.size(); ++i) {
    threads[i].join();
    (*cloud) += *(thread_cloud[i]);
  }
  for (auto && detected_ids_in_thread : thread_detected_ids) {
    for (const auto & id : detected_ids_in_thread) {
      detected_objects_.emplace_back(geometry_ids_[id]);
    }
  }

  for (const auto & id : geometry_ids_) {
    rtcDetachGeometry(scene_, id.first);
  }

  geometry_ids_.clear();
  primitive_ptrs_.clear();
#endif  // DO_NOT_COMPILE_THIS_

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  return pointcloud_msg;
}
}  // namespace simple_sensor_simulator
