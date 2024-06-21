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

#include "test_detection_sensor.hpp"

#include <limits>

#include "../../utils/expect_eq_macros.hpp"

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and UNKNOWN
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the standard detection functionality when an entity is detected because it is close to Ego.
 */
TEST_F(DetectionSensorTest, update_unknownSubtype)
{
  initializeEntityStatus("unknown", EntityType::VEHICLE, EntitySubtype::UNKNOWN);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::UNKNOWN);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and CAR
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the standard detection functionality when an entity is detected because it is close to Ego.
 */
TEST_F(DetectionSensorTest, update_carSubtype)
{
  initializeEntityStatus("car", EntityType::VEHICLE, EntitySubtype::CAR);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::CAR);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and TRUCK
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the standard detection functionality when an entity is detected because it is close to Ego.
 */
TEST_F(DetectionSensorTest, update_truckSubtype)
{
  initializeEntityStatus("truck", EntityType::VEHICLE, EntitySubtype::TRUCK);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::TRUCK);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and BUS
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the standard detection functionality when an entity is detected because it is close to Ego.
 */
TEST_F(DetectionSensorTest, update_busSubtype)
{
  initializeEntityStatus("bus", EntityType::VEHICLE, EntitySubtype::BUS);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::BUS);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and TRAILER
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the standard detection functionality when an entity is detected because it is close to Ego.
 */
TEST_F(DetectionSensorTest, update_trailerSubtype)
{
  initializeEntityStatus("trailer", EntityType::VEHICLE, EntitySubtype::TRAILER);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::TRAILER);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and
 * MOTORCYCLE entity positioned closer to Ego than the range parameter (in configuration) - the goal
 * is to test the standard detection functionality when an entity is detected because it is close to
 * Ego.
 */
TEST_F(DetectionSensorTest, update_motorcycleSubtype)
{
  initializeEntityStatus("motorcycle", EntityType::VEHICLE, EntitySubtype::MOTORCYCLE);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::MOTORCYCLE);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and BICYCLE
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the standard detection functionality when an entity is detected because it is close to Ego
 */
TEST_F(DetectionSensorTest, update_bicycleSubtype)
{
  initializeEntityStatus("bicycle", EntityType::VEHICLE, EntitySubtype::BICYCLE);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::BICYCLE);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = true (in configuration), no probability of lost (configuration) and
 * PEDESTRIAN entity positioned closer to Ego than the range parameter (in configuration) - the goal
 * is to test the standard detection functionality when an entity is detected because it is close to
 * Ego
 */
TEST_F(DetectionSensorTest, update_pedestrianSubtype)
{
  initializeEntityStatus("pedestrian", EntityType::PEDESTRIAN, EntitySubtype::PEDESTRIAN);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::PEDESTRIAN);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, a
 * substantial positive delay and filter_by_range = true (in configuration) no probability of lost
 * (configuration) and some entity positioned closer to Ego than the range parameter (in
 * configuration) - the goal is to test the simulated detection delay correctness
 */
TEST_F(DetectionSensorTest, update_detectionDelay)
{
  config_.set_object_recognition_delay(0.5);
  detection_sensor_ = std::make_unique<DetectionSensor<DetectedObjectsMsg>>(
    0.0, config_, detected_objects_publisher_, ground_truth_objects_publisher_);

  initializeEntityStatus("pedestrian", EntityType::PEDESTRIAN, EntitySubtype::PEDESTRIAN);

  // Initial update: before the delay period, should not detect the object
  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  // Check that no message has been received yet
  EXPECT_EQ(received_msg_, nullptr);

  // Advance the simulation time to after the delay period
  current_simulation_time_ += 0.6;  // 0.6 seconds, greater than the 0.5-second delay
  current_ros_time_ = rclcpp::Time(current_simulation_time_, RCL_ROS_TIME);

  // Update again: after the delay period, should detect the object
  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::PEDESTRIAN);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no  delay
 * and filter_by_range = true (in configuration) 100% probability of lost (configuration) and some
 * entity positioned closer to Ego than the range parameter (in configuration) - the goal is to test
 * the simulated malfunction when the message is not being delivered
 */
TEST_F(DetectionSensorTest, update_looseAllData)
{
  config_.set_probability_of_lost(1.0);  // 100% probability of lost

  detection_sensor_ = std::make_unique<DetectionSensor<DetectedObjectsMsg>>(
    0.0, config_, detected_objects_publisher_, ground_truth_objects_publisher_);

  initializeEntityStatus("pedestrian", EntityType::PEDESTRIAN, EntitySubtype::PEDESTRIAN);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality. Test update process correctness with no position noise, no delay
 * and filter_by_range = false (in configuration) no probability of lost (configuration) and some
 * entity in lidar_detected_entity vector - the goal is to test detection based on lidar
 * functionality
 */
TEST_F(DetectionSensorTest, update_lidarBasedDetection)
{
  config_.set_detect_all_objects_in_range(false);

  detection_sensor_ = std::make_unique<DetectionSensor<DetectedObjectsMsg>>(
    0.0, config_, detected_objects_publisher_, ground_truth_objects_publisher_);

  initializeEntityStatus("pedestrian", EntityType::PEDESTRIAN, EntitySubtype::PEDESTRIAN);

  detection_sensor_->update(
    current_simulation_time_, status_, current_ros_time_, lidar_detected_entities_);

  // Spin the node to process callbacks
  rclcpp::spin_some(node_);

  ASSERT_NE(received_msg_, nullptr);
  ASSERT_EQ(received_msg_->objects.size(), static_cast<size_t>(1));
  EXPECT_EQ(received_msg_->objects[0].classification[0].label, ObjectClassification::PEDESTRIAN);
  EXPECT_POSITION_NEAR(
    received_msg_->objects[0].kinematics.pose_with_covariance.pose.position, entity_pose_.position,
    std::numeric_limits<double>::epsilon());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
