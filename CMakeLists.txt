cmake_minimum_required(VERSION 3.10)
project(rpp_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rpp_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPose.srv"
  "srv/ToggleFilterProcessing.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_generate_interfaces creates a custom target named after the package.
set(core_library_name rpp_localization_core)
set(rpp_interface_dir "$ENV{HOME}/.rpp/interfaces/cpp")
option(
  RPP_LOCALIZATION_ENABLE_PLUGIN_TESTS
  "Build tests that require locally generated RPP plugin-type headers"
  OFF)
option(
  RPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS
  "Build launch tests that replay the checked-in localization rosbag"
  OFF)

add_library(${core_library_name} SHARED
  src/core/filter_utilities.cpp
  src/filters/extended_kalman_filter.cpp
  src/filters/unscented_kalman_filter.cpp
  src/models/constant_acceleration_model.cpp
  src/models/navigation_model_base.cpp
  src/ros/ros_filter.cpp
  src/ros/ros_filter_base.cpp
  src/ros/ros_filter_utilities.cpp
)

target_include_directories(${core_library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${rpp_interface_dir}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${core_library_name}
  PUBLIC
    Eigen3::Eigen
    rpp_cpp::rpp_cpp
    "${cpp_typesupport_target}"
)
target_link_libraries(${core_library_name}
  PUBLIC
    ament_index_cpp::ament_index_cpp
    angles::angles
    diagnostic_msgs::diagnostic_msgs
    diagnostic_updater::diagnostic_updater
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    std_msgs::std_msgs
    std_srvs::std_srvs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
)

add_executable(localization_node src/ros/nodes/localization_node.cpp)
add_executable(ekf_node src/ros/nodes/ekf_node.cpp)
add_executable(ukf_node src/ros/nodes/ukf_node.cpp)
target_link_libraries(localization_node PRIVATE ${core_library_name})
target_link_libraries(ekf_node PRIVATE ${core_library_name})
target_link_libraries(ukf_node PRIVATE ${core_library_name})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_core_compatibility test/test_core_compatibility.cpp)
  target_link_libraries(test_core_compatibility ${core_library_name})

  ament_add_gtest(
    test_constant_acceleration_model_parity
    test/test_constant_acceleration_model_parity.cpp)
  target_link_libraries(test_constant_acceleration_model_parity ${core_library_name})

  ament_add_gtest(test_ukf test/test_ukf.cpp)
  target_link_libraries(test_ukf ${core_library_name})

  if(RPP_LOCALIZATION_ENABLE_PLUGIN_TESTS)
    ament_add_gtest(
      test_localization_plugin_boundary
      test/test_localization_plugin_boundary.cpp)
    target_link_libraries(test_localization_plugin_boundary ${core_library_name})
    target_include_directories(test_localization_plugin_boundary PRIVATE plugins/filters)

    ament_add_gtest(test_plugin_headers test/test_plugin_headers.cpp)
    target_link_libraries(test_plugin_headers ${core_library_name})
    target_include_directories(test_plugin_headers PRIVATE plugins/filters plugins/models)
  endif()

  if(RPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_ekf_rosbag_integration.launch.py TIMEOUT 90)
  endif()
endif()

install(TARGETS ${core_library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS localization_node ekf_node ukf_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/rpp_localization/core
  include/rpp_localization/filters
  include/rpp_localization/models
  include/rpp_localization/ros
  DESTINATION include/rpp_localization
)
install(DIRECTORY .rppws DESTINATION share/${PROJECT_NAME})
install(DIRECTORY plugin_types plugins DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})
install(FILES plugins.json DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp angles Eigen3 rpp_cpp rclcpp diagnostic_msgs diagnostic_updater geometry_msgs
  nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
  rosidl_default_runtime
)
ament_package()
