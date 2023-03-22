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

#define RAYCASTER_DEBUG 1
#if RAYCASTER_DEBUG == 1
#define CATCH_RGL_ERROR(err) { if (err != 0) {std::cout << "RGL error code: " << err << std::endl; breaker();} }
void breaker()   // put your breakpoint here
{
}
#else
#define CATCH_RGL_ERROR(err) err
#endif

rgl_mat3x4f getRglIdentity()
{
  rgl_mat3x4f mat;

  mat.value[0][0] = 1;
  mat.value[1][0] = 0;
  mat.value[2][0] = 0;

  mat.value[0][1] = 0;
  mat.value[1][1] = 1;
  mat.value[2][1] = 0;

  mat.value[0][2] = 0;
  mat.value[1][2] = 0;
  mat.value[2][2] = 1;

  mat.value[0][3] = 0;
  mat.value[1][3] = 0;
  mat.value[2][3] = 0;

  return mat;
}

void printMat(const Eigen::Matrix3d & mat)
{
  printf("%.2f %.2f %.2f\n", mat(0, 0), mat(0, 1), mat(0, 2));
  printf("%.2f %.2f %.2f\n", mat(1, 0), mat(1, 1), mat(1, 2));
  printf("%.2f %.2f %.2f\n", mat(2, 0), mat(2, 1), mat(2, 2));
}

void printMat(const rgl_mat3x4f & mat)
{
  printf("%.2f %.2f %.2f %.2f\n", mat.value[0][0], mat.value[0][1], mat.value[0][2], mat.value[0][3]);
  printf("%.2f %.2f %.2f %.2f\n", mat.value[1][0], mat.value[1][1], mat.value[1][2], mat.value[1][3]);
  printf("%.2f %.2f %.2f %.2f\n", mat.value[2][0], mat.value[2][1], mat.value[2][2], mat.value[2][3]);
}

/**
 * Sets given rotation to RGL matrix with possible correction for RGL base orientation aligned with Z axis
 * @param entity_tf reference to the matrix where changes are applied
 * @param rotation rotation quaternion to apply
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatRotation(rgl_mat3x4f & entity_tf, geometry_msgs::msg::Quaternion rotation, const bool correct_rgl_bias = true)
{
/*
  geometry_msgs::msg::Vector3 rpy1;
  rpy1.x = M_PI/4;
  rpy1.y = M_PI/4;
  rpy1.z = 0;
  geometry_msgs::msg::Vector3 rpy2;
  rpy2.x = 0;
  rpy2.y = 0;
  rpy2.z = M_PI/2;
  const auto q1 = quaternion_operation::convertEulerAngleToQuaternion(rpy1);
  const auto q2 = quaternion_operation::convertEulerAngleToQuaternion(rpy2);
  const auto r1 = quaternion_operation::rotation(q1, q2);   // rotates around local frame
  const auto r2 = q2*q1;   // rotates around global frame
  std::cout << "Rot1: " << r1.x << ' ' << r1.y << ' ' << r1.z << ' ' << r1.w << std::endl;
  std::cout << "Rot2: " << r2.x << ' ' << r2.y << ' ' << r2.z << ' ' << r2.w << std::endl;
*/

  if (correct_rgl_bias) {
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0;
    rpy.y = M_PI/2;
    rpy.z = 0;
    const auto bias = quaternion_operation::convertEulerAngleToQuaternion(rpy);
    rotation = quaternion_operation::rotation(rotation, bias);   // rotates about a local frame
    // rotation = bias * rotation;   // rotates about a global frame
  }
  const auto rotation_mat = quaternion_operation::getRotationMatrix(rotation);

  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      // no need to convert from column-major to row-major as () operator already does this
      entity_tf.value[row][col] = static_cast<float>(rotation_mat(row, col));
    }
  }
}

/**
 * Converts quaternion rotation to RGL matrix rotation (position is set to [0, 0, 0])
 * @param rotation rotation quaternion
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 * @return rotation in matrix rgl_mat3x4f
 */
rgl_mat3x4f getRglMatRotation(const geometry_msgs::msg::Quaternion & rotation, const bool correct_rgl_bias = true)
{
  rgl_mat3x4f tf = getRglIdentity();
  setRglMatRotation(tf, rotation, correct_rgl_bias);
  return tf;
}

/**
 * Sets given position to RGL matrix
 * @param entity_tf reference to the matrix to modify
 * @param position position to set in matrix
 */
void setRglMatPosition(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Point & position)
{
  entity_tf.value[0][3] = position.x;
  entity_tf.value[1][3] = position.y;
  entity_tf.value[2][3] = position.z;
}

/**
 * Converts pose (both position and orientation) to RGL matrix
 * @param entity_tf RGL matrix to change values
 * @param pose pose with values to transfer into RGL matrix
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatPose(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose, const bool correct_rgl_bias = true)
{
  setRglMatRotation(entity_tf, pose.orientation, correct_rgl_bias);
  setRglMatPosition(entity_tf, pose.position);
}

/**
 * Inverts position and converts it to RGL matrix
 * @param entity_tf RGL matrix to modify
 * @param pose pose with position to invert
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatPositionInv(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose)
{
  auto pose_inv = pose;
  {
    pose_inv.position.x = -pose.position.x;
    pose_inv.position.y = -pose.position.y;
    pose_inv.position.z = -pose.position.z;
  }
  setRglMatPosition(entity_tf, pose_inv.position);
}

/**
 * Inverts rotation and converts it to RGL matrix
 * @param entity_tf RGL matrix to modify
 * @param pose pose with rotation to invert
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatRotationInv(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose, const bool correct_rgl_bias = true)
{
  auto pose_inv = pose;
  {
    pose_inv.orientation.x = -pose.orientation.x;
    pose_inv.orientation.y = -pose.orientation.y;
    pose_inv.orientation.z = -pose.orientation.z;
  }
  setRglMatRotation(entity_tf, pose_inv.orientation, correct_rgl_bias);
}

/**
 * Initialize vertices of a box with given dimensions
 * @param vertices reference to an array to write values to
 * @param depth depth of a box
 * @param width width of a box
 * @param height height of a box
 */
void initBoxVertices(rgl_vec3f (&vertices)[8], float depth, float width, float height)
{
  vertices[0].value[0] = -0.5 * depth;
  vertices[0].value[1] = -0.5 * width;
  vertices[0].value[2] = -0.5 * height;

  vertices[1].value[0] = -0.5 * depth;
  vertices[1].value[1] = -0.5 * width;
  vertices[1].value[2] = +0.5 * height;

  vertices[2].value[0] = -0.5 * depth;
  vertices[2].value[1] = +0.5 * width;
  vertices[2].value[2] = -0.5 * height;

  vertices[3].value[0] = -0.5 * depth;
  vertices[3].value[1] = +0.5 * width;
  vertices[3].value[2] = +0.5 * height;

  vertices[4].value[0] = +0.5 * depth;
  vertices[4].value[1] = -0.5 * width;
  vertices[4].value[2] = -0.5 * height;

  vertices[5].value[0] = +0.5 * depth;
  vertices[5].value[1] = -0.5 * width;
  vertices[5].value[2] = +0.5 * height;

  vertices[6].value[0] = +0.5 * depth;
  vertices[6].value[1] = +0.5 * width;
  vertices[6].value[2] = -0.5 * height;

  vertices[7].value[0] = +0.5 * depth;
  vertices[7].value[1] = +0.5 * width;
  vertices[7].value[2] = +0.5 * height;
}

/**
 * Initialize indices of a box (works with initBoxVertices())
 * @param indices reference to an array to write values to
 */
void initBoxIndices(rgl_vec3i (&indices)[12])
{
  indices[0].value[0] = 0;
  indices[0].value[1] = 1;
  indices[0].value[2] = 2;

  indices[1].value[0] = 1;
  indices[1].value[1] = 3;
  indices[1].value[2] = 2;

  indices[2].value[0] = 4;
  indices[2].value[1] = 6;
  indices[2].value[2] = 5;

  indices[3].value[0] = 5;
  indices[3].value[1] = 6;
  indices[3].value[2] = 7;

  indices[4].value[0] = 0;
  indices[4].value[1] = 4;
  indices[4].value[2] = 1;

  indices[5].value[0] = 1;
  indices[5].value[1] = 4;
  indices[5].value[2] = 5;

  indices[6].value[0] = 2;
  indices[6].value[1] = 3;
  indices[6].value[2] = 6;

  indices[7].value[0] = 3;
  indices[7].value[1] = 7;
  indices[7].value[2] = 6;

  indices[8].value[0] = 0;
  indices[8].value[1] = 2;
  indices[8].value[2] = 4;

  indices[9].value[0] = 2;
  indices[9].value[1] = 6;
  indices[9].value[2] = 4;

  indices[10].value[0] = 1;
  indices[10].value[1] = 5;
  indices[10].value[2] = 3;

  indices[11].value[0] = 3;
  indices[11].value[1] = 5;
  indices[11].value[2] = 7;
}


namespace simple_sensor_simulator
{
Raycaster::Raycaster()
: primitive_ptrs_(0),
  device_(rtcNewDevice(nullptr)),
  scene_(rtcNewScene(device_)),
  engine_(seed_gen_())
{
  int32_t major, minor, patch;
  CATCH_RGL_ERROR(rgl_get_version_info(&major, &minor, &patch));
  std::cout << "====================" << major << "." << minor << "." << patch << std::endl;
}

Raycaster::Raycaster(std::string embree_config)
: primitive_ptrs_(0),
  device_(rtcNewDevice(embree_config.c_str())),
  scene_(rtcNewScene(device_)),
  engine_(seed_gen_())
{
}

Raycaster::~Raycaster()
{
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);
}

/**
 * Creates a RGL mesh and entity of an object and stores it, if entity with provided name already exists this does nothing
 * @param name Name of the entity to create
 * @param depth Depth of a box model used in lidar simulation
 * @param width Width of a box model used in lidar simulation
 * @param height Height of a box model used in lidar simulation
 */
void Raycaster::addEntity(const std::string & name, float depth, float width, float height)
{
  const auto it = entities_.find(name);   // if entity exists skip adding
  if (it != entities_.end()) {
    return;
  }
  rgl_mesh_t mesh = nullptr;
  rgl_vec3f vertices[8];
  initBoxVertices(vertices, depth, width, height);   // TODO check whether box is oriented in way it should
  rgl_vec3i indices[12];
  initBoxIndices(indices);
  CATCH_RGL_ERROR(rgl_mesh_create(&mesh, vertices, 8, indices, 12));   // Saves handle to a mesh in the pointer 'mesh'
  rgl_entity_t entity = nullptr;
  CATCH_RGL_ERROR(rgl_entity_create(&entity, nullptr, mesh));   // Saves handle to an entity in the pointer 'entity'
  entities_[name] = entity;   // Adds handle to entity to the map, every element in entity (including mesh referenced) are allocated by `rgl_` functions
}

/**
 * Sets pose of an entity with provided name
 * @param name Name of an entity to relocate
 * @param pose Desired entity pose
 * @return True if an entity with provided name exists, else false
 */
bool Raycaster::setEntityPose(const std::string & name, const geometry_msgs::msg::Pose & pose)
{
  const auto it = entities_.find(name);
  if (it == entities_.end()) {
    return false;
  }
  rgl_mat3x4f entity_tf;
  setRglMatPose(entity_tf, pose);
  CATCH_RGL_ERROR(rgl_entity_set_pose(it->second, &entity_tf));
  return true;
}

void Raycaster::setDirection(
  const simulation_api_schema::LidarConfiguration & configuration, double horizontal_angle_start,
  double horizontal_angle_end)
{
  std::vector<double> vertical_angles;
  // for (double v = -3.1415; v < 3.1415; v += 0.01) {
  // for (double v = 0; v < 3.1415*2; v += 0.01) {
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
    rotation_matrices_rgl_.push_back(getRglMatRotation(q));   // Add ray rotations in RGL format
  }
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
        rpy.y = vertical_angle;// + M_PI/2;
        rpy.z = horizontal_angle;
        std::cout << "RPY: " << rpy.x << ' ' << rpy.y << ' ' << rpy.z << std::endl;
        auto quat = quaternion_operation::convertEulerAngleToQuaternion(rpy);
        directions.emplace_back(quat);
      }
    }
    directions_ = directions;
    std::cout << "==================================SIZE: " << directions.size() << std::endl;
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
  double max_distance, double min_distance)
{
  detected_objects_ = {};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto & pair : primitive_ptrs_) {
    auto id = pair.second->addToScene(device_, scene_);
    geometry_ids_.insert({id, pair.first});
  }

  static rgl_node_t use_rays = nullptr, lidar_pose = nullptr, raytrace = nullptr, compact = nullptr,
                    lidar_position_inv = nullptr, lidar_rotation_inv = nullptr;
  static bool init = true;

  /* Set matrix to transform rays into origin tf */
  rgl_mat3x4f lidar_pose_tf = getRglIdentity();
  rgl_mat3x4f lidar_position_inv_tf = getRglIdentity();
  rgl_mat3x4f lidar_rotation_inv_tf = getRglIdentity();
  /* do not correct RGL bias, as this pose is fed directly to RGL to transform all rays (account for lidar pose)
  and everything added to rgl is already corrected */
  setRglMatPose(lidar_pose_tf, origin, false);
  /* TODO optimal solution would be to cast rays with lidar position at [0, 0, 0] (higher floating point precision),
  this requires to change positions of every entity to account for created offset */
  setRglMatPositionInv(lidar_position_inv_tf, origin);
  setRglMatRotationInv(lidar_rotation_inv_tf, origin, false);

  /* initialize rays only once */
  if (init) {
    CATCH_RGL_ERROR(rgl_node_rays_from_mat3x4f(&use_rays, rotation_matrices_rgl_.data(), rotation_matrices_rgl_.size()));
  }
  /* set transformation of ego to every ray every time raycast is called */
  CATCH_RGL_ERROR(rgl_node_rays_transform(&lidar_pose, &lidar_pose_tf));
  CATCH_RGL_ERROR(rgl_node_raytrace(&raytrace, nullptr, static_cast<float>(max_distance)));
  CATCH_RGL_ERROR(rgl_node_points_compact(&compact));
  CATCH_RGL_ERROR(rgl_node_points_transform(&lidar_position_inv, &lidar_position_inv_tf));
  CATCH_RGL_ERROR(rgl_node_points_transform(&lidar_rotation_inv, &lidar_rotation_inv_tf));

  /* create graph only once */
  if (init) {
    CATCH_RGL_ERROR(rgl_graph_node_add_child(use_rays, lidar_pose));
    CATCH_RGL_ERROR(rgl_graph_node_add_child(lidar_pose, raytrace));
    CATCH_RGL_ERROR(rgl_graph_node_add_child(raytrace, compact));
    CATCH_RGL_ERROR(rgl_graph_node_add_child(compact, lidar_position_inv));
    CATCH_RGL_ERROR(rgl_graph_node_add_child(lidar_position_inv, lidar_rotation_inv));
  }
  init = false;

  CATCH_RGL_ERROR(rgl_graph_run(compact));

  int32_t out_count, out_size_of;
  CATCH_RGL_ERROR(rgl_graph_get_result_size(lidar_rotation_inv, RGL_FIELD_XYZ_F32, &out_count, &out_size_of));

  static std::vector<rgl_vec3f> results;

  if (out_count > 0) {
    results.resize(static_cast<size_t>(out_count));
    CATCH_RGL_ERROR(rgl_graph_get_result_data(lidar_rotation_inv, RGL_FIELD_XYZ_F32, results.data()));
    for (const auto & result : results) {
      pcl::PointXYZI p;
      {
        p.x = result.value[0];
        p.y = result.value[1];
        p.z = result.value[2];
      }
      cloud->emplace_back(p);
      // should add ID
    }
    std::cout << "==================== HITS: " << out_count << " ====================" << std::endl;
    std::cout << "First hit: "
              << (*results.begin()).value[0] << ' '
              << (*results.begin()).value[1] << ' '
              << (*results.begin()).value[2] << std::endl;
    std::cout << "Last hit: "
              << (*results.rbegin()).value[0] << ' '
              << (*results.rbegin()).value[1] << ' '
              << (*results.rbegin()).value[2] << std::endl;
  } else {
    std::cout << "==================== NO HITS ====================" << std::endl;
  }

/*
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
*/

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  return pointcloud_msg;
}
}  // namespace simple_sensor_simulator
