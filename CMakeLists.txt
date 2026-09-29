cmake_minimum_required(VERSION 3.8)
project(rpp_localization)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

#set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rosbag2_cpp)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

set(library_name rpp_localization_lib)
set(use_python_bindings OFF)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLL.srv"
  "srv/GetState.srv"
  "srv/SetDatum.srv"
  "srv/SetPose.srv"
  "srv/ToggleFilterProcessing.srv"
  "srv/ToLL.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
  ADD_LINTER_TESTS
)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(
  ${library_name}
  src/models/constant_acc_model.cpp
  src/filters/ekf.cpp
  src/filters/ukf.cpp
  src/filters/nav_filter.cpp
  src/models/nav_model_base.cpp
  src/core/filter_utilities.cpp
  # src/ros/navsat_transform.cpp
  # src/filters/estimator.cpp
  src/ros/ros_filter.cpp
  src/ros/ros_filter_base.cpp
  src/ros/ros_filter_utilities.cpp
  src/ros/rosbag_msg_provider.cpp
  src/ros/rosbag_filter.cpp
  # src/ros/listener.cpp
  src/inekf/inertial_process.cpp
  src/inekf/twist_sensor.cpp
  src/inekf/pose_sensor.cpp
  src/inekf/inekf.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${library_name} "${cpp_typesupport_target}")

if (use_python_bindings)

  find_package(pybind11)
  find_package(PythonLibs)
  find_package(ament_cmake_python REQUIRED)

  target_sources(${library_name} PRIVATE
    src/models/python_model.cpp)

  include_directories(
    ${pybind11_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
  )
  # target_link_libraries(${library_name} pybind11) # pybind is header-only
  ament_python_install_package(${PROJECT_NAME}_py)

endif()

add_executable(
  ekf_node
  src/ros/nodes/ekf_node.cpp
)

add_executable(
  ukf_node
  src/ros/nodes/ukf_node.cpp
)

add_executable(
  inekf_node
  src/ros/nodes/inekf_node.cpp
)

add_executable(
  rosbag_filter
  src/ros/nodes/rosbag_filter_node.cpp
)

# add_executable(
#   navsat_transform_node
#   src/ros/nodes/navsat_transform_node.cpp
# )

# add_executable(
#   rpp_localization_listener_node
#   src/ros/nodes/listener_node.cpp
# )

target_link_libraries(
  ${library_name}
  ${GeographicLib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

ament_target_dependencies(
  ${library_name}
  angles
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
  rosbag2_cpp
)

target_link_libraries(
  ekf_node
  ${library_name}
)

target_link_libraries(
  ukf_node
  ${library_name}
)

target_link_libraries(
  inekf_node
  ${library_name}
)

target_link_libraries(
  rosbag_filter
  ${library_name}
)

# target_link_libraries(
#   navsat_transform_node
#   ${library_name}
# )

# ament_target_dependencies(
#   navsat_transform_node
#   rclcpp
# )

# target_link_libraries(
#   rpp_localization_listener_node
#   ${library_name}
# )

# ament_target_dependencies(
#   rpp_localization_listener_node
#   rclcpp
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  #### FILTER BASE TESTS ####
  ament_add_gtest(filter_base-test test/test_filter_base.cpp)
  target_link_libraries(filter_base-test ${library_name})
  target_include_directories(filter_base-test PRIVATE src)


  #### DIAGNOSTICS TESTS ####
  ament_add_gtest_executable(test_filter_base_diagnostics_timestamps
                    test/test_filter_base_diagnostics_timestamps.cpp)
  target_link_libraries(test_filter_base_diagnostics_timestamps ${library_name})

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_filter_base_diagnostics_timestamps "${cpp_typesupport_target}")

  add_dependencies(test_filter_base_diagnostics_timestamps ekf_node)
  ament_add_test(test_filter_base_diagnostics_timestamps
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_filter_base_diagnostics_timestamps.launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

  #### EKF TESTS ######
  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf ${library_name})

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_ekf "${cpp_typesupport_target}")

  ament_add_gtest_executable(
    test_ekf_node_interfaces
    test/test_se_node_interfaces.cpp)
  target_link_libraries(
    test_ekf_node_interfaces
    ${library_name})

  rosidl_get_typesupport_target(
    cpp_typesupport_target
    "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(
    test_ekf_node_interfaces
    "${cpp_typesupport_target}")

  add_dependencies(
    test_ekf_node_interfaces
    ekf_node)
  ament_add_test(
    test_ekf_node_interfaces
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_se_node_interfaces.launch.py"
    ENV NODE_TYPE=ekf
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

  #### INEKF TESTS ######
  


  #### UKF TESTS #####
  ament_add_gtest(test_ukf test/test_ukf.cpp)
  target_link_libraries(test_ukf ${library_name})

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_ukf "${cpp_typesupport_target}")

  ament_add_gtest_executable(
    test_ukf_node_interfaces
    test/test_se_node_interfaces.cpp)
  target_link_libraries(
    test_ukf_node_interfaces
    ${library_name})

  rosidl_get_typesupport_target(
    cpp_typesupport_target
    "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(
    test_ukf_node_interfaces
    "${cpp_typesupport_target}")

  add_dependencies(
    test_ukf_node_interfaces
    ukf_node)
  ament_add_test(
    test_ukf_node_interfaces
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_se_node_interfaces.launch.py"
    ENV NODE_TYPE=ukf
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

  #### DATA TESTS #####
 ament_add_gtest(test_ekf_localization_node_bag1 test/test_ekf_localization_node_bag1.cpp)
 target_include_directories(test_ekf_localization_node_bag1 PRIVATE src)
 target_link_libraries(test_ekf_localization_node_bag1 ${library_name})

 ament_add_gtest(test_ekf_localization_node_bag2 test/test_ekf_localization_node_bag2.cpp)
 target_include_directories(test_ekf_localization_node_bag2 PRIVATE src)
 target_link_libraries(test_ekf_localization_node_bag2 ${library_name})

 ament_add_gtest(test_ekf_localization_node_bag3 test/test_ekf_localization_node_bag3.cpp)
 target_include_directories(test_ekf_localization_node_bag3 PRIVATE src)
 target_link_libraries(test_ekf_localization_node_bag3 ${library_name})

 ament_add_gtest(test_ukf_localization_node_bag1 test/test_ukf_localization_node_bag1.cpp)
 target_include_directories(test_ukf_localization_node_bag1 PRIVATE src)
 target_link_libraries(test_ukf_localization_node_bag1 ${library_name})

 ament_add_gtest(test_ukf_localization_node_bag2 test/test_ukf_localization_node_bag2.cpp)
 target_include_directories(test_ukf_localization_node_bag2 PRIVATE src)
 target_link_libraries(test_ukf_localization_node_bag2 ${library_name})

 ament_add_gtest(test_ukf_localization_node_bag3 test/test_ukf_localization_node_bag3.cpp)
 target_include_directories(test_ukf_localization_node_bag3 PRIVATE src)
 target_link_libraries(test_ukf_localization_node_bag3 ${library_name})

 ament_add_gtest(test_inekf_localization_node_bag1 test/test_inekf_localization_node_bag1.cpp)
 target_include_directories(test_inekf_localization_node_bag1 PRIVATE src)
 target_link_libraries(test_inekf_localization_node_bag1 ${library_name})
# INSTALL(CODE "execute_process( \
#     COMMAND ${CMAKE_COMMAND} -E create_symlink \
#     ${CMAKE_CURRENT_SOURCE_DIR}/test1 \
#     ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/test/test1   \
#     )"
# )

#   #### RLE/RLL TESTS #####
#   ament_add_gtest_executable(test_rpp_localization_estimator
#                   test/test_rpp_localization_estimator.cpp)
#   target_link_libraries(test_rpp_localization_estimator ${library_name})
#   rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
#   target_link_libraries(test_rpp_localization_estimator "${cpp_typesupport_target}")
#   ament_add_test(test_rpp_localization_estimator
#     GENERATE_RESULT_FOR_RETURN_CODE_ZERO
#     TIMEOUT 100
#     COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rpp_localization_estimator.launch.py"
#     WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
#   )

#   ament_add_gtest_executable(test_ros_rpp_localization_listener
#                   test/test_ros_rpp_localization_listener.cpp)
#   target_link_libraries(test_ros_rpp_localization_listener ${library_name})
#   rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
#   target_link_libraries(test_ros_rpp_localization_listener "${cpp_typesupport_target}")
#   ament_add_gtest_executable(test_ros_rpp_localization_listener_publisher
#                   test/test_ros_rpp_localization_listener_publisher.cpp)
#   target_link_libraries(test_ros_rpp_localization_listener_publisher ${library_name})
#   rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
#   target_link_libraries(test_ros_rpp_localization_listener_publisher "${cpp_typesupport_target}")
#   ament_add_test(test_ros_rpp_localization_listener
#     GENERATE_RESULT_FOR_RETURN_CODE_ZERO
#     TIMEOUT 100
#     COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_rpp_localization_listener.launch.py"
#     WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
#   )

  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  install(
    TARGETS
    filter_base-test
    test_filter_base_diagnostics_timestamps
    test_ekf
    test_ekf_node_interfaces
    test_ukf
    test_ukf_node_interfaces
    test_ekf_localization_node_bag1
    test_ekf_localization_node_bag2
    test_ekf_localization_node_bag3
    test_ukf_localization_node_bag1
    test_ukf_localization_node_bag2
    test_ukf_localization_node_bag3
    test_inekf_localization_node_bag1
    # test_rpp_localization_estimator
    # test_ros_rpp_localization_listener
    # test_ros_rpp_localization_listener_publisher
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install test launch config files.
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )
endif()

install(
  TARGETS
  # navsat_transform_node
  ekf_node
  ukf_node
  inekf_node
  rosbag_filter
  # rpp_localization_listener_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
  USE_SOURCE_PERMISSIONS)

# Install params config files.
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
