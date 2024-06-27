find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME})

add_subdirectory(src/sensor_simulation/lidar)
add_subdirectory(src/sensor_simulation/primitives)
add_subdirectory(src/sensor_simulation/occupancy_grid)
add_subdirectory(src/sensor_simulation/detection_sensor)
add_subdirectory(src/sensor_simulation/traffic_lights)
