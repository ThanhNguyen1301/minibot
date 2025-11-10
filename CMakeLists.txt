cmake_minimum_required(VERSION 3.5)
project(minibot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(velocity_publisher demo_publisher_node/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher
  rclcpp
  geometry_msgs
)

install(TARGETS
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds demo_publisher_node maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
