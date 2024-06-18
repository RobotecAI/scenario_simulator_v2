add_subdirectory(src/sensor_simulation/lidar)
add_subdirectory(src/sensor_simulation/primitives)
add_subdirectory(src/sensor_simulation/occupancy_grid)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

ament_add_gtest(test_scenario_simulator src/test_scenario_simulator.cpp)
target_link_libraries(test_scenario_simulator simple_sensor_simulator_component ${Protobuf_LIBRARIES})
