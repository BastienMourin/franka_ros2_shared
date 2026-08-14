cmake_minimum_required(VERSION 3.8)
project(my_cartesian_impedance_controller_cmp_dq)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dq_operations REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(${PROJECT_NAME} SHARED
  src/my_cartesian_impedance_controller_cmp_dq.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  ${dq_operations_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcl_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
  realtime_tools
  ament_index_cpp
  dq_operations
)

pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
  realtime_tools
  pinocchio
  dq_operations
)

ament_package()
