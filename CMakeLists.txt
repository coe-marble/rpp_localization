cmake_minimum_required(VERSION 3.10)
project(rpp_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTING "Build rpp_localization tests" OFF)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(rpp_cpp REQUIRED)
find_package(rpp_interfaces REQUIRED)
find_package(rpp_ros2 REQUIRED)
find_package(rpp_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(core_library_name rpp_localization_core)
option(
  RPP_LOCALIZATION_ENABLE_PLUGIN_TESTS
  "Build tests that require generated RPP headers and a registered plugin library"
  OFF)
option(
  RPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS
  "Build launch tests that replay the checked-in localization rosbag"
  OFF)
option(
  RPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS
  "Compare the generic estimator with robot_localization on recorded bags"
  OFF)
option(
  RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS
  "Compile opt-in benchmark timing and enable instrumented numerical variants"
  OFF)

add_library(rpp_localization_numerics INTERFACE)
add_library(rpp_localization::numerics ALIAS rpp_localization_numerics)
set_target_properties(rpp_localization_numerics PROPERTIES EXPORT_NAME numerics)
target_compile_features(rpp_localization_numerics INTERFACE cxx_std_17)
target_include_directories(rpp_localization_numerics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rpp_localization_numerics INTERFACE Eigen3::Eigen angles::angles)

# Numerical implementations are header-only; retain the shared target for ROS adapters.
add_library(${core_library_name} SHARED
  src/ros/estimator_script_node.cpp
  src/ros/navigation_interface.cpp
)
target_compile_definitions(${core_library_name} PUBLIC
  RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=$<BOOL:${RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS}>)

target_include_directories(${core_library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${core_library_name}
  PUBLIC
    rpp_localization::numerics
    rpp_cpp::rpp_cpp
    rpp_interfaces::rpp_interfaces
    rpp_ros2::rpp_ros2
    Threads::Threads
    ${rcl_interfaces_TARGETS}
    ${rpp_ros_interfaces_TARGETS}
)
target_link_libraries(${core_library_name}
  PUBLIC
    ament_index_cpp::ament_index_cpp
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    std_srvs::std_srvs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)

add_executable(localization_node src/ros/nodes/localization_node.cpp)
add_executable(ekf_node src/ros/nodes/ekf_node.cpp)
add_executable(ukf_node src/ros/nodes/ukf_node.cpp)
add_executable(estimator_script_node src/ros/nodes/estimator_script_node.cpp)
target_link_libraries(localization_node PRIVATE ${core_library_name})
target_link_libraries(ekf_node PRIVATE ${core_library_name})
target_link_libraries(ukf_node PRIVATE ${core_library_name})
target_link_libraries(estimator_script_node PRIVATE ${core_library_name})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_parity_metrics
    test/robot_localization_parity/test_parity_metrics.py)

  ament_add_gtest(test_estimation test/test_estimation.cpp)
  target_compile_definitions(test_estimation PRIVATE
    RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS=$<BOOL:${RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS}>)
  target_link_libraries(test_estimation rpp_localization::numerics Threads::Threads)

  ament_add_gtest(test_numerics_linkage
    test/test_numerics_linkage.cpp
    test/numerics_linkage_peer.cpp
  )
  target_link_libraries(test_numerics_linkage rpp_localization::numerics)

  if(RPP_LOCALIZATION_ENABLE_PLUGIN_TESTS)
    ament_add_gtest(test_shared_estimator test/test_shared_estimator.cpp)
    target_link_libraries(test_shared_estimator ${core_library_name})
    target_include_directories(test_shared_estimator PRIVATE plugins/filters plugins/models)
    target_compile_definitions(test_shared_estimator PRIVATE
      RPP_LOCALIZATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

    ament_add_gtest(test_estimator_script_node test/test_estimator_script_node.cpp)
    target_link_libraries(test_estimator_script_node ${core_library_name})
    target_compile_definitions(test_estimator_script_node PRIVATE
      RPP_LOCALIZATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  endif()

  if(RPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_ekf_rosbag_integration.launch.py TIMEOUT 150)
  endif()
  if(RPP_LOCALIZATION_ENABLE_BAG_BENCHMARKS)
    find_package(launch_testing_ament_cmake REQUIRED)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND RPP_LOCALIZATION_ENABLE_BENCHMARK_METRICS)
      add_library(rpp_localization_worker_metrics SHARED
        test/robot_localization_parity/worker_allocations.cpp)
      target_include_directories(rpp_localization_worker_metrics PRIVATE include)
      target_link_libraries(rpp_localization_worker_metrics PRIVATE Threads::Threads)
      install(TARGETS rpp_localization_worker_metrics LIBRARY DESTINATION lib)
    endif()
    # robot_localization is a runtime test dependency in package.xml. The launch
    # test resolves its executable through the ament index; no libraries are linked.
    add_launch_test(test/robot_localization_parity/test_robot_localization_parity.launch.py TIMEOUT 7200)
  endif()
endif()

install(TARGETS ${core_library_name} rpp_localization_numerics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS localization_node ekf_node ukf_node estimator_script_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/rpp_localization/estimation
  include/rpp_localization/models
  include/rpp_localization/ros
  DESTINATION include/rpp_localization
)
install(DIRECTORY .rppws DESTINATION share/${PROJECT_NAME})
install(DIRECTORY plugins DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(FILES plugins.json DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp angles Eigen3 Threads rpp_cpp rpp_interfaces rpp_ros2 rpp_ros_interfaces rclcpp rcl_interfaces geometry_msgs
  nav_msgs sensor_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
)
ament_package()
