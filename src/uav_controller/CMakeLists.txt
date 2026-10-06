cmake_minimum_required(VERSION 3.8)
project(uav_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs mavros_msgs message_generation nav_msgs roscpp sensor_msgs
  std_msgs uav_mission_msgs visualization_msgs
)
find_package(Eigen3 REQUIRED)

add_message_files(FILES TrajectoryCommand.msg TrajectorySetpoint.msg FlightStatus.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs uav_mission_msgs)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_controller_core
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs roscpp
                 sensor_msgs std_msgs uav_mission_msgs visualization_msgs
  DEPENDS EIGEN3
)

# Reusable ROS-free modules.
add_library(uav_controller_core src/geometric_controller.cpp src/flight_controller.cpp)
target_include_directories(uav_controller_core PUBLIC include ${EIGEN3_INCLUDE_DIRS})

# ROS adapters.
add_executable(geometric_controller_node src/geometric_controller_node.cpp)
target_link_libraries(geometric_controller_node uav_controller_core ${catkin_LIBRARIES})
add_executable(fixed_point_pub src/fixed_point_pub.cpp)
target_link_libraries(fixed_point_pub ${catkin_LIBRARIES})
add_executable(trajectory_rviz_node src/trajectory_rviz_node.cpp)
target_link_libraries(trajectory_rviz_node ${catkin_LIBRARIES})
foreach(node geometric_controller_node fixed_point_pub trajectory_rviz_node)
  add_dependencies(${node} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(${node} PRIVATE include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
endforeach()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(flight_controller_test test/flight_controller_test.cpp)
  if(TARGET flight_controller_test)
    target_link_libraries(flight_controller_test uav_controller_core)
  endif()
  catkin_install_python(PROGRAMS test/position_step_ros_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/position_step_ros.test DEPENDENCIES geometric_controller_node trajectory_rviz_node)
endif()

install(TARGETS geometric_controller_node fixed_point_pub trajectory_rviz_node
                uav_controller_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
