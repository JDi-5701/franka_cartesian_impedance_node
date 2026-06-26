cmake_minimum_required(VERSION 3.16)
project(franka_cartesian_impedance_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
# GoToPose.srv now lives in the interface-only package (no libfranka) so the operator PC
# can build just the interface to call the services.
find_package(franka_cartesian_impedance_msgs REQUIRED)

add_executable(cartesian_impedance_node src/cartesian_impedance_node.cpp)
target_include_directories(cartesian_impedance_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(cartesian_impedance_node
  rclcpp geometry_msgs sensor_msgs std_srvs franka_cartesian_impedance_msgs)
target_link_libraries(cartesian_impedance_node Franka::Franka)

add_executable(admittance_node src/admittance_node.cpp)
target_include_directories(admittance_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(admittance_node rclcpp geometry_msgs sensor_msgs std_msgs)
target_link_libraries(admittance_node Franka::Franka)

# Torque-level Cartesian impedance (factorization damping + nullspace); computes joint
# torques ourselves instead of using the robot's built-in Cartesian impedance.
add_executable(cartesian_impedance_torque_node src/cartesian_impedance_torque_node.cpp)
target_include_directories(cartesian_impedance_torque_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(cartesian_impedance_torque_node rclcpp geometry_msgs sensor_msgs)
target_link_libraries(cartesian_impedance_torque_node Franka::Franka)

install(TARGETS cartesian_impedance_node admittance_node cartesian_impedance_torque_node
        DESTINATION lib/${PROJECT_NAME})

# gripper_check.py: standalone Franka gripper connectivity/latency test (robot side).
install(PROGRAMS
  scripts/gripper_check.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
