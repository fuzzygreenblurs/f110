cmake_minimum_required(VERSION 3.8)
project(aeb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
include_directories("/usr/include/eigen3")


#include header files directory
include_directories(include)

# define the executable and source files
add_executable(safety_node src/safety_node.cpp)

# link dependencies to executable
ament_target_dependencies(safety_node
	rclcpp
	sensor_msgs
	nav_msgs
	ackermann_msgs
)

# install the executable
install(TARGETS safety_node
	DESTINATION lib/${PROJECT_NAME}
)

# install the header files
install(DIRECTORY include/
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
