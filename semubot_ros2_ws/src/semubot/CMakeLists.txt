cmake_minimum_required(VERSION 3.8)
project(semubot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# Install URDF files
install(
  DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install Python scripts
install(PROGRAMS
  scripts/cmd_vel_publisher.py
  scripts/cmd_vel_serial.py
  scripts/cmd_vel_to_joint_states.py
  scripts/odom_broadcaster.py
  #scripts/cmd_ultrasonic_range_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)


set(BUILD_TESTING OFF)

ament_package()