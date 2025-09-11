
# wbqp_controller/CMakeLists.txt
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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wbqp_codegen_vendor REQUIRED)

include_directories(
  include
)

add_executable(wbqp_controller
  src/wbqp_controller.cpp
)

ament_target_dependencies(wbqp_controller
  rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros wbqp_codegen_vendor
)

# Link the three static libraries exported by wbqp_codegen_vendor
# Also pulls their public include directories.
# target_link_libraries(wbqp_controller
#   wbqp_init
#   wbqp_solve
#   whole_body_jacobian
# )

# Pull include dirs from the vendor
target_include_directories(wbqp_controller PRIVATE
  ${wbqp_codegen_vendor_INCLUDE_DIRS}
)

# Link with full paths exported by the vendor
target_link_libraries(wbqp_controller
  ${wbqp_codegen_vendor_LIBRARIES}
)

install(TARGETS wbqp_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
