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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../expect_eq_macros.hpp"

static const std::string map_path = "/map/lanelet2_map.osm";
static const std::string with_road_shoulder_map_path = "/map/with_road_shoulder/lanelet2_map.osm";
static const std::string empty_map_path = "/map/empty/lanelet2_map.osm";
static const std::string four_track_map_path = "/map/four_track_highway/lanelet2_map.osm";

auto makeHdMapUtilsInstance(const std::string relative_path = map_path) -> hdmap_utils::HdMapUtils
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + relative_path;
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  return hdmap_utils::HdMapUtils(path, origin);
}

auto makePoint(const double x, const double y, const double z = 0.0) -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

auto makeBoundingBox(const double center_y = 0.0) -> traffic_simulator_msgs::msg::BoundingBox
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 1.0;
  bbox.center.y = center_y;
  bbox.dimensions.x = 4.0;
  bbox.dimensions.y = 2.0;
  bbox.dimensions.z = 1.5;
  return bbox;
}

auto makePose(
  geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation = geometry_msgs::msg::Quaternion())
  -> geometry_msgs::msg::Pose
{
  geometry_msgs::msg::Pose pose;
  pose.position = position;
  pose.orientation = orientation;
  return pose;
}

auto makeSmallBoundingBox(const double center_y = 0.0) -> traffic_simulator_msgs::msg::BoundingBox
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = center_y;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  bbox.dimensions.z = 1.0;
  return bbox;
}

auto makeQuaternionFromYaw(const double yaw) -> geometry_msgs::msg::Quaternion
{
  geometry_msgs::msg::Vector3 v;
  v.z = yaw;
  return quaternion_operation::convertEulerAngleToQuaternion(v);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(HdMapUtils, Construct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  ASSERT_NO_THROW(hdmap_utils.toMapBin());
}

TEST(HdMapUtils, matchToLane)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = 0.0;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(120659, 1, 0)).pose,
      bbox, false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 120659);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34411, 1, 0)).pose,
      bbox, false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 34411);
  }
}

TEST(HdMapUtils, AlongLaneletPose)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30)
      .lanelet_id,
    34513);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(
        traffic_simulator::helper::constructLaneletPose(34513, 0, 0),
        hdmap_utils.getLaneletLength(34513) + 10.0)
      .lanelet_id,
    34510);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30.0)
      .s,
    30.0);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30.0)
      .lanelet_id,
    34513);

  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), -10.0)
      .lanelet_id,
    34684);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), -10.0)
      .s,
    hdmap_utils.getLaneletLength(34684) - 10.0);
}

/**
 * @note Testcase for lanelet pose canonicalization when s < 0
 * Following lanelets: 34576 -> 34570 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34576, s=-22 + length of 34570 + length of 34576)
 */
TEST(HdMapUtils, CanonicalizeNegative)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = -22;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34564, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s +
                                            hdmap_utils.getLaneletLength(34570) +
                                            hdmap_utils.getLaneletLength(34576));
}

/**
 * @note Testcase for lanelet pose canonicalization when s > length of lanelet pose
 * Following lanelets: 34981 -> 34585 -> 34579
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 */
TEST(HdMapUtils, CanonicalizePositive)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 30;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s -
                                            hdmap_utils.getLaneletLength(34585) -
                                            hdmap_utils.getLaneletLength(34981));
}

/**
 * @note Testcase for lanelet pose canonicalization when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is suppose to be the same
 */
TEST(HdMapUtils, Canonicalize)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 2;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s);
}

/**
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s < 0
 * Following lanelets: 34576 -> 34570 -> 34564
 *                     34981 -> 34636 -> 34564
 *                     34600 -> 34648 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34575, s=-22 + length of 34570 + length of 34576)
 *                               (id=34981, s=-22 + length of 34636 + length of 34981)
 *                               (id=34600, s=-22 + length of 34648 + length of 34600)
 */
TEST(HdMapUtils, CanonicalizeAllNegative)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = -22;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34564, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.getAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s +
                                        hdmap_utils.getLaneletLength(34570) +
                                        hdmap_utils.getLaneletLength(34576));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34981);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, non_canonicalized_lanelet_s +
                                        hdmap_utils.getLaneletLength(34636) +
                                        hdmap_utils.getLaneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34600);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, non_canonicalized_lanelet_s +
                                        hdmap_utils.getLaneletLength(34648) +
                                        hdmap_utils.getLaneletLength(34600));
}

/**
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s > length of lanelet pose
 * Following lanelets: 34981 -> 34585 -> 34579
 *                     34981 -> 34636 -> 34564
 *                     34981 -> 34651 -> 34630
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 *                               (id=34564, s=30 - length of 34636 - length of 34981)
 *                               (id=34630, s=30 - length of 34651 - length of 34981)
 */
TEST(HdMapUtils, CanonicalizeAllPositive)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 30;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.getAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s -
                                        hdmap_utils.getLaneletLength(34585) -
                                        hdmap_utils.getLaneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34564);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, non_canonicalized_lanelet_s -
                                        hdmap_utils.getLaneletLength(34636) -
                                        hdmap_utils.getLaneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34630);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, non_canonicalized_lanelet_s -
                                        hdmap_utils.getLaneletLength(34651) -
                                        hdmap_utils.getLaneletLength(34981));
}

/**
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is suppose to be the same
 */
TEST(HdMapUtils, CanonicalizeAll)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 2;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.getAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(1));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s);
}

TEST(HdMapUtils, Construct_invalid)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "invalid_path";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  EXPECT_THROW(hdmap_utils::HdMapUtils hdmap_utils(path, origin), std::runtime_error);
}

TEST(HdMapUtils, filterLaneletIds_emptyIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Ids ids = {};
  const char * subtype = "crosswalk";
  auto filtered = hdmap_utils.filterLaneletIds(ids, subtype);

  EXPECT_TRUE(filtered.empty());
}

TEST(HdMapUtils, filterLaneletIds_invalidIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_invalid_0 = 10000000;
  lanelet::Id id_invalid_1 = 10000001;
  lanelet::Id id_invalid_2 = 10000002;
  lanelet::Ids ids = {id_invalid_0, id_invalid_1, id_invalid_2};
  const char * subtype = "crosswalk";

  EXPECT_THROW(auto filtered = hdmap_utils.filterLaneletIds(ids, subtype), std::runtime_error);
}

TEST(HdMapUtils, filterLaneletIds_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk_0 = 34399;
  lanelet::Id id_crosswalk_1 = 34385;
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_crosswalk_0, id_crosswalk_1, id_road_0, id_road_1};
  const char * subtype = "crosswalk";

  auto filtered = hdmap_utils.filterLaneletIds(ids, subtype);

  EXPECT_EQ(filtered.size(), 2);
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_0) != filtered.end());
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_1) != filtered.end());
}

TEST(HdMapUtils, filterLaneletIds_invalidSubtype)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk_0 = 34399;
  lanelet::Id id_crosswalk_1 = 34385;
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_crosswalk_0, id_crosswalk_1, id_road_0, id_road_1};
  const char * subtype = "invalid_subtype";

  auto filtered = hdmap_utils.filterLaneletIds(ids, subtype);

  EXPECT_TRUE(filtered.empty());
}

TEST(HdMapUtils, getNearbyLaneletIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3807.34, 73817.95);
  const double distance_threshold = 10.0;
  size_t search_count = 100;
  auto lanelets = hdmap_utils.getNearbyLaneletIds(point, distance_threshold, search_count);

  lanelet::Ids nearbyLanelets = {34795, 120660, 34507, 34468, 120659, 34606};
  EXPECT_EQ(lanelets, nearbyLanelets);
}

TEST(HdMapUtils, getNearbyLaneletIds_unsuccessful)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3826.26, 73837.32);
  const double distance_threshold = 10.0;
  size_t search_count = 100;
  auto lanelets = hdmap_utils.getNearbyLaneletIds(point, distance_threshold, search_count);

  EXPECT_TRUE(lanelets.empty());
}

TEST(HdMapUtils, getNearbyLaneletIds_crosswalkIncluded)
{
  // refer to 1st issue at the end of this file
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3768.01, 73750.97);
  const double distance_threshold = 0.0;
  bool include_crosswalk = true;
  size_t search_count = 100;
  auto lanelets =
    hdmap_utils.getNearbyLaneletIds(point, distance_threshold, include_crosswalk, search_count);

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34633;
  EXPECT_EQ(lanelets.size(), 2);
  EXPECT_TRUE(std::find(lanelets.begin(), lanelets.end(), id_crosswalk) != lanelets.end());
  EXPECT_TRUE(std::find(lanelets.begin(), lanelets.end(), id_road) != lanelets.end());
}

TEST(HdMapUtils, getNearbyLaneletIds_crosswalkNotIncluded)
{
  // refer to 1st issue at the end of this file
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3768.01, 73750.97);
  const double distance_threshold = 0.0;
  bool include_crosswalk = false;
  size_t search_count = 100;
  auto lanelets =
    hdmap_utils.getNearbyLaneletIds(point, distance_threshold, include_crosswalk, search_count);

  lanelet::Id id_road = 34633;
  EXPECT_EQ(lanelets.size(), 1);
  EXPECT_EQ(lanelets[0], id_road);
}

TEST(HdMapUtils, getNearbyLaneletIds_crosswalkUnsuccessful)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3826.26, 73837.32);
  const double distance_threshold = 10.0;
  bool include_crosswalk = true;
  size_t search_count = 100;
  auto lanelets =
    hdmap_utils.getNearbyLaneletIds(point, distance_threshold, include_crosswalk, search_count);

  EXPECT_TRUE(lanelets.empty());
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_intersects)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34633;
  auto distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road, id_crosswalk);

  EXPECT_TRUE(distance.has_value());
  EXPECT_GT(distance.value(), 0.0);
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_disjoint)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34579;
  auto distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road, id_crosswalk);

  EXPECT_FALSE(distance.has_value());
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_invalidLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road_invalid = 1000000;

  std::optional<double> distance;
  EXPECT_THROW(
    distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road_invalid, id_crosswalk),
    std::runtime_error);
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_invalidCrosswalkLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk_invalid = 1000000;
  lanelet::Id id_road = 34600;

  std::optional<double> distance;
  EXPECT_THROW(
    distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road, id_crosswalk_invalid),
    std::runtime_error);
}

TEST(HdMapUtils, matchToLane_includeCrosswalk)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34399, 1, 0)).pose,
      bbox, true);
    EXPECT_TRUE(id.has_value());
    EXPECT_EQ(id.value(), 34399);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34399, 1, 0)).pose,
      bbox, false);
    if (id.has_value()) {
      EXPECT_NE(id.value(), 34399);
    }
  }
}

TEST(HdMapUtils, matchToLane_noMatch)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34392, 0, 0)).pose,
      bbox, false);
    EXPECT_FALSE(id.has_value());
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34378, 0, 0)).pose,
      bbox, false);
    EXPECT_FALSE(id.has_value());
  }
}

TEST(HdMapUtils, getSpeedLimit_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_road_0, id_road_1};

  double speed_limit = hdmap_utils.getSpeedLimit(ids);

  const double true_limit = 50.0 / 3.6;
  const double eps = 0.01;
  EXPECT_NEAR(speed_limit, true_limit, eps);
}

TEST(HdMapUtils, getSpeedLimit_crosswalk)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk_0 = 34399;
  lanelet::Id id_crosswalk_1 = 34385;
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_crosswalk_0, id_crosswalk_1, id_road_0, id_road_1};

  double speed_limit = hdmap_utils.getSpeedLimit(ids);

  const double true_limit = 0.0 / 3.6;
  const double eps = 0.01;
  EXPECT_NEAR(speed_limit, true_limit, eps);
}

TEST(HdMapUtils, getSpeedLimit_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Ids ids = {};

  EXPECT_THROW(static_cast<void>(hdmap_utils.getSpeedLimit(ids)), std::runtime_error);
}

TEST(HdMapUtils, getClosestLaneletId_near)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id = 120659;
  auto position = makePoint(3818.91, 73787.95);
  auto pose = makePose(position);
  const double distance_threshold = 1.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), id);
}

TEST(HdMapUtils, getClosestLaneletId_away)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto position = makePoint(3775.82, 73743.29);
  auto pose = makePose(position);
  const double distance_threshold = 1.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_FALSE(result.has_value());
}

TEST(HdMapUtils, getClosestLaneletId_crosswalkCloserButExcluded)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34639;
  auto position = makePoint(3774.73, 73744.38);
  auto pose = makePose(position);
  const double distance_threshold = 5.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), id_road);
  EXPECT_NE(result.value(), id_crosswalk);
}

TEST(HdMapUtils, getClosestLaneletId_onlyCrosswalkNearButExcluded)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34639;
  auto position = makePoint(3774.73, 73744.38);
  auto pose = makePose(position);
  const double distance_threshold = 5.0;
  const bool include_crosswalk = true;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), id_crosswalk);
  EXPECT_NE(result.value(), id_road);
}

TEST(HdMapUtils, getClosestLaneletId_emptyMap)
{
  auto hdmap_utils = makeHdMapUtilsInstance(empty_map_path);
  auto position = makePoint(3.0, 5.0);
  auto pose = makePose(position);
  const double distance_threshold = 7.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_FALSE(result.has_value());
}

TEST(HdMapUtils, getPreviousLaneletIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34468;
  const lanelet::Id prev_lanelet = 120660;

  const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet));
  }
}

TEST(HdMapUtils, getNextLaneletIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 120660;
  const lanelet::Id next_lanelet = 34468;

  const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet));
  }
}

TEST(HdMapUtils, getPreviousLaneletIds_RoadShoulder)
{
  std::string map_path(with_road_shoulder_map_path);
  auto hdmap_utils = makeHdMapUtilsInstance(map_path);
  const lanelet::Id curr_lanelet = 34768;
  const lanelet::Id prev_lanelet = 34696;

  const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet));
  }
}

TEST(HdMapUtils, getNextLaneletIds_RoadShoulder)
{
  std::string map_path(with_road_shoulder_map_path);
  auto hdmap_utils = makeHdMapUtilsInstance(map_path);
  const lanelet::Id curr_lanelet = 34696;
  const lanelet::Id next_lanelet = 34768;

  const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet));
  }
}

TEST(HdMapUtils, getNextLaneletIds_multipleNext)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34468;
  const lanelet::Id next_lanelet_0 = 34438;
  const lanelet::Id next_lanelet_1 = 34465;
  lanelet::Ids next_lanelets = {next_lanelet_0, next_lanelet_1};

  auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet);

  std::sort(next_lanelets.begin(), next_lanelets.end());
  std::sort(result_ids.begin(), result_ids.end());

  EXPECT_EQ(next_lanelets, result_ids);
}

TEST(HdMapUtils, getPreviousLaneletIds_multiplePrevious)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34462;
  const lanelet::Id prev_lanelet_0 = 34411;
  const lanelet::Id prev_lanelet_1 = 34465;
  lanelet::Ids prev_lanelets = {prev_lanelet_0, prev_lanelet_1};

  auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet);

  std::sort(prev_lanelets.begin(), prev_lanelets.end());
  std::sort(result_ids.begin(), result_ids.end());

  EXPECT_EQ(prev_lanelets, result_ids);
}

TEST(HdMapUtils, getNextLaneletIds_direction)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34468;
  const lanelet::Id next_lanelet_left = 34438;
  const lanelet::Id next_lanelet_straight = 34465;

  {
    std::string turn_direction = "left";
    const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet_left));
    }
  }
  {
    std::string turn_direction = "straight";
    const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet_straight));
    }
  }
}

TEST(HdMapUtils, getPreviousLaneletIds_direction)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34462;
  const lanelet::Id prev_lanelet_left = 34411;
  const lanelet::Id prev_lanelet_straight = 34465;

  {
    std::string turn_direction = "left";
    const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_left));
    }
  }
  {
    std::string turn_direction = "straight";
    const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_straight));
    }
  }
}

TEST(HdMapUtils, isInRoute_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34468;
  lanelet::Ids route = {};

  EXPECT_FALSE(hdmap_utils.isInRoute(lanelet_id, route));
}

TEST(HdMapUtils, isInRoute_notOnRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34468;
  const lanelet::Id route_part_0 = 34741;
  const lanelet::Id route_part_1 = 34850;
  const lanelet::Id route_part_2 = 34603;
  const lanelet::Id route_part_3 = 34777;
  lanelet::Ids route = {route_part_0, route_part_1, route_part_2, route_part_3};

  EXPECT_FALSE(hdmap_utils.isInRoute(lanelet_id, route));
}

TEST(HdMapUtils, isInRoute_onRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id route_part_0 = 34741;
  const lanelet::Id route_part_1 = 34850;
  const lanelet::Id route_part_2 = 34603;
  const lanelet::Id route_part_3 = 34777;
  const lanelet::Id lanelet_id = route_part_1;
  lanelet::Ids route = {route_part_0, route_part_1, route_part_2, route_part_3};

  EXPECT_TRUE(hdmap_utils.isInRoute(lanelet_id, route));
}

TEST(HdMapUtils, isInLanelet_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = 10;

  EXPECT_TRUE(hdmap_utils.isInLanelet(lanelet_id, s));
}

TEST(HdMapUtils, isInLanelet_after)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = hdmap_utils.getLaneletLength(lanelet_id) + 5.0;

  EXPECT_FALSE(hdmap_utils.isInLanelet(lanelet_id, s));
}

TEST(HdMapUtils, isInLanelet_before)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = -5;

  EXPECT_FALSE(hdmap_utils.isInLanelet(lanelet_id, s));
}

TEST(HdMapUtils, toMapPoints_correctPoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const std::vector<double> s{10.0, 20.0, 30.0};

  const auto points = hdmap_utils.toMapPoints(lanelet_id, s);

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));
  EXPECT_POINT_NEAR(points[0], makePoint(3768.7, 73696.2, 1.9), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3759.8, 73691.6, 2.1), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3750.9, 73687.1, 2.3), 0.1);
}

TEST(HdMapUtils, toMapPoints_negativeS)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const std::vector<double> s{-10.0, -20.0, -30.0};

  const auto points = hdmap_utils.toMapPoints(lanelet_id, s);

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));

  EXPECT_POINT_NEAR(points[0], makePoint(3786.5, 73705.3, 1.5), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3795.4, 73709.9, 1.3), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3804.3, 73714.5, 1.1), 0.1);
}

TEST(HdMapUtils, toMapPoints_sLargerThanLaneletLength)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;

  const auto lanelet_length = hdmap_utils.getLaneletLength(lanelet_id);
  const std::vector<double> s{lanelet_length + 10.0, lanelet_length + 20.0, lanelet_length + 30.0};

  const auto points = hdmap_utils.toMapPoints(lanelet_id, s);

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));
  EXPECT_POINT_NEAR(points[0], makePoint(3725.8, 73674.2, 3.0), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3716.9, 73669.6, 3.1), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3708.0, 73665.0, 3.3), 0.1);
}

TEST(HdMapUtils, toMapPoints_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;

  std::vector<geometry_msgs::msg::Point> points;

  EXPECT_NO_THROW(points = hdmap_utils.toMapPoints(lanelet_id, {}));

  EXPECT_TRUE(points.empty());
}

TEST(HdMapUtils, toMapPose_onlyOffset)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = 10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;
  pose.offset = 0.5;

  const auto map_pose = hdmap_utils.toMapPose(pose);

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3768.9, 73695.8, 1.9), makeQuaternionFromYaw(-2.667)), 0.1);
}

TEST(HdMapUtils, toMapPose_additionalRotation)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = 10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;
  pose.rpy.z = M_PI_4;

  const auto map_pose = hdmap_utils.toMapPose(pose);

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose,
    makePose(makePoint(3768.7, 73696.2, 1.9), makeQuaternionFromYaw(-2.667 + M_PI_4)), 0.1);
}

TEST(HdMapUtils, toMapPose_negativeS)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = -10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;

  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(map_pose = hdmap_utils.toMapPose(pose));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3784.0, 73707.8, 1.5), makeQuaternionFromYaw(-1.595)), 0.1);
}

TEST(HdMapUtils, toMapPose_sLargerThanLaneletLength)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = hdmap_utils.getLaneletLength(lanelet_id) + 10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;

  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(map_pose = hdmap_utils.toMapPose(pose));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3724.9, 73678.1, 2.7), makeQuaternionFromYaw(2.828)), 0.1);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_straight)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 199;
  const lanelet::Id end_lanelet = start_lanelet;
  const auto direction = traffic_simulator::lane_change::Direction::STRAIGHT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(end_lanelet, result_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_leftNoChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 199;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_leftChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 200;
  const lanelet::Id end_lanelet = 199;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_rightNoChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 202;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_rightChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 200;
  const lanelet::Id end_lanelet = 201;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2LeftPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 201;
  const lanelet::Id end_lanelet = 199;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2LeftNotPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 200;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2RightPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 200;
  const lanelet::Id end_lanelet = 202;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2RightNotPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 201;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift0)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);
  const lanelet::Id start_lanelet = 201;
  const lanelet::Id end_lanelet = 201;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;
  const uint8_t shift = 0;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getTrafficLightIds_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Ids traffic_lights = {34802, 34836};

  auto result_traffic_lights = hdmap_utils.getTrafficLightIds();

  std::sort(result_traffic_lights.begin(), result_traffic_lights.end());
  EXPECT_EQ(result_traffic_lights, traffic_lights);
}

TEST(HdMapUtils, getTrafficLightIds_noTrafficLight)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_map_path);

  auto result_traffic_lights = hdmap_utils.getTrafficLightIds();

  EXPECT_EQ(result_traffic_lights.size(), static_cast<size_t>(0));
}

TEST(HdMapUtils, getTrafficLightBulbPosition_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id light_id = 34802;
  const double epsilon = 0.1;

  {
    const std::string color = "green";
    const auto actual_bulb_position = makePoint(3761.05, 73755.30, 5.35);

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), actual_bulb_position, epsilon);
  }

  {
    const std::string color = "yellow";
    const auto actual_bulb_position = makePoint(3760.60, 73755.07, 5.35);

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), actual_bulb_position, epsilon);
  }

  {
    const std::string color = "red";
    const auto actual_bulb_position = makePoint(3760.16, 73754.87, 5.35);

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), actual_bulb_position, epsilon);
  }

  {
    const std::string color = "pink";

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_FALSE(return_bulb_position.has_value());
  }
}

TEST(HdMapUtils, getTrafficLightBulbPosition_invalidTrafficLight)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id light_id = 1000003;

  {
    const std::string color = "red";

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_FALSE(return_bulb_position.has_value());
  }
}

/*
ISSUES:
1: 288, missing predicate if first is closer than distance threshold.
  Differning from line 265.
*/