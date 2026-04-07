cmake_minimum_required(VERSION 3.8)
project(my_cartesian_velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(my_cartesian_velocity_controller SHARED
  src/my_cartesian_velocity_controller.cpp
)

target_include_directories(my_cartesian_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}  
  ${PINOCCHIO_INCLUDE_DIRS} 
)

target_link_libraries(my_cartesian_velocity_controller
  pinocchio::pinocchio
)

ament_target_dependencies(my_cartesian_velocity_controller
  rclcpp 
  controller_interface 
  hardware_interface 
  pluginlib 
  rclcpp_lifecycle 
  geometry_msgs    
  realtime_tools
  ament_index_cpp
)

pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

install(
  TARGETS my_cartesian_velocity_controller
  EXPORT export_my_cartesian_controller
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
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(my_cartesian_velocity_controller)
ament_export_dependencies(
  rclcpp 
  controller_interface 
  hardware_interface 
  pluginlib 
  rclcpp_lifecycle 
  geometry_msgs 
  realtime_tools
  pinocchio 
)

ament_package()
