#include <gtest/gtest.h>

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../catalogs.hpp"
#include "../entity_helper_functions.hpp"
#include "../expect_eq_macros.hpp"

auto makePedestrianBoundingBox() -> traffic_simulator_msgs::msg::BoundingBox
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = 0.0;
  bbox.center.z = 1.0;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  bbox.dimensions.z = 2.0;
  return bbox;
}

auto makePedestrianParameters()
{
  traffic_simulator_msgs::msg::PedestrianParameters parameters;
  parameters.name = "pedestrian_entity";
  parameters.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
  parameters.bounding_box = makePedestrianBoundingBox();
  return parameters;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader("traffic_simulator", "entity_behavior::BehaviorPluginBase");
  auto plugin = loader.createSharedInstance("do_nothing_plugin/DoNothingPlugin");
  testing::InitGoogleTest(&argc, argv);
  const bool test_result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return test_result;
}

TEST(PedestrianEntity, setAccelerationLimit_correct)
{
  const double init_speed = 0.0;
  const std::string entity_name = "bob_entity";
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, init_speed, entity_name);
  auto parameters = makePedestrianParameters();

  traffic_simulator::entity::PedestrianEntity bob(entity_name, status, hdmap_utils_ptr, parameters);

  bob.startNpcLogic();
  // plugin problem
}