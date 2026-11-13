cmake_minimum_required(VERSION 3.20)
project(neo_relayboard_v2-2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    common/include
)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  neo_msgs2
  neo_srvs2
  std_srvs
)

ament_export_include_directories(common/include)

add_library(${PROJECT_NAME} common/src/NeoRelayBoardNode.cpp 
  common/src/RelayBoardClient.cpp common/src/SerialIO.cpp common/src/StrUtil.cpp)

ament_export_libraries(${PROJECT_NAME})

add_executable(neo_relayboard_node src/neo_relayboard_v2_node.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${neo_msgs2_TARGETS}
  ${neo_srvs2_TARGETS}
  ${std_srvs_TARGETS}
)

target_link_libraries(neo_relayboard_node PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${neo_msgs2_TARGETS}
  ${neo_srvs2_TARGETS}
  ${std_srvs_TARGETS}
)

install(DIRECTORY common/include/
   DESTINATION common/include/
)

install(TARGETS neo_relayboard_node
DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch
    
  DESTINATION share/${PROJECT_NAME})

ament_package()