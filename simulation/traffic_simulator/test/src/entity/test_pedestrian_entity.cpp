#include <gtest/gtest.h>

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../catalogs.hpp"
#include "../expect_eq_macros.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

auto makeHdMapUtilsSharedPointer() -> std::shared_ptr<hdmap_utils::HdMapUtils>
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.9037067912303;
  origin.longitude = 139.9337945139059;
  return std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
}

auto makeCanonicalizedLaneletPose(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils, const lanelet::Id id = 120659,
  const double s = 0.0, const double offset = 0.0)
  -> traffic_simulator::lanelet_pose::CanonicalizedLaneletPose
{
  return traffic_simulator::lanelet_pose::CanonicalizedLaneletPose(
    traffic_simulator::helper::constructLaneletPose(id, s, offset), hdmap_utils);
}

auto makePedestrianBoundingBox() -> traffic_simulator_msgs::msg::BoundingBox
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = 0.0;
  bbox.center.z = 0.5;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  bbox.dimensions.z = 2.0;
  return bbox;
}

auto makeEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "pedestrian_entity") -> traffic_simulator::EntityStatus
{
  traffic_simulator::EntityStatus entity_status;
  entity_status.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  entity_status.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
  entity_status.time = 0.0;
  entity_status.name = name;
  entity_status.bounding_box = bbox;
  geometry_msgs::msg::Twist twist;
  entity_status.action_status =
    traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0);
  entity_status.lanelet_pose_valid = true;
  entity_status.lanelet_pose = static_cast<traffic_simulator::LaneletPose>(pose);
  entity_status.pose = hdmap_utils->toMapPose(entity_status.lanelet_pose).pose;
  return entity_status;
}

auto makeCanonicalizedEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "pedestrian_entity")
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    makeEntityStatus(hdmap_utils, pose, bbox, speed, name), hdmap_utils);
}

auto makePedestrianParameters()
{
  traffic_simulator_msgs::msg::PedestrianParameters parameters;
  parameters.name = "pedestrian";
  parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
  parameters.bounding_box = makePedestrianBoundingBox();
  return parameters;
}

TEST(PedestrianEntity, setAccelerationLimit_correct)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  auto parameters = makePedestrianParameters();

  const std::string plugin_name = traffic_simulator::entity::PedestrianEntity::BuiltinBehavior::defaultBehavior();
  auto bob = traffic_simulator::entity::PedestrianEntity("pedestrian_entity", status, hdmap_utils_ptr, parameters, plugin_name);

  bob.startNpcLogic();

}