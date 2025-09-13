cmake_minimum_required(VERSION 3.16)
project(wbqp_controller)

# Consume catkin vars if a tool passes them, to avoid "unused variable" warnings
set(CATKIN_INSTALL_INTO_PREFIX_ROOT OFF CACHE BOOL "Compatibility var (ignored)")
set(CATKIN_SYMLINK_INSTALL OFF CACHE BOOL "Compatibility var (ignored)")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wbqp_codegen_vendor REQUIRED)

include_directories(
  include
)

# Install controller executable
add_executable(wbqp_controller
  src/wbqp_controller.cpp
)

ament_target_dependencies(wbqp_controller
  rclcpp sensor_msgs geometry_msgs std_msgs std_srvs tf2_ros wbqp_codegen_vendor
)

# --- Menu node target ---
add_executable(wbqp_menu
  src/wbqp_menu.cpp
)
ament_target_dependencies(wbqp_menu
  rclcpp std_srvs
)

# --- TCP pose converter node target ---
add_executable(tcp_pose_converter 
  src/tcp_pose_converter.cpp
)
ament_target_dependencies(tcp_pose_converter
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

target_include_directories(tcp_pose_converter PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

# Pull include dirs from the vendor
target_include_directories(wbqp_controller PRIVATE
  ${wbqp_codegen_vendor_INCLUDE_DIRS}
)

# Link with full paths exported by the vendor
target_link_libraries(wbqp_controller
  ${wbqp_codegen_vendor_LIBRARIES}
)

# Install targets
install(TARGETS wbqp_controller wbqp_menu tcp_pose_converter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch + config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
