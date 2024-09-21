cmake_minimum_required(VERSION 3.8)
project(otv_bf7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


set(ament_dependencies
  rclcpp
  geometry_msgs
)

add_executable(draw_node src/draw_node.cpp)
ament_target_dependencies(draw_node  ${ament_dependencies})
add_executable(iranyitas src/iranyitas.cpp)
ament_target_dependencies(iranyitas  ${ament_dependencies})

install(TARGETS draw_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS iranyitas 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 

  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
