cmake_minimum_required(VERSION 2.8.3)
project(chapter5_tutorials)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  tf
)
add_message_files(FILES
  odom_arduino.msg
)
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_broadcast src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcast ${catkin_LIBRARIES})

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})

add_executable(laser src/laser.cpp)
target_link_libraries(laser ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(base_controller src/base_controller.cpp)
target_link_libraries(base_controller ${catkin_LIBRARIES})





add_executable(odom_arduino
  src/odom_arduino.cpp
)
add_dependencies(odom_arduino ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_arduino
  ${catkin_LIBRARIES}
)

add_executable(dummy_tick_publis
  src/dummy_tick_publisher.cpp
)
add_dependencies(dummy_tick_publis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dummy_tick_publis
  ${catkin_LIBRARIES}
)

add_executable(odomo_arduino_class
  src/odomo_arduino_class.cpp
)
add_dependencies(odomo_arduino_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odomo_arduino_class
  ${catkin_LIBRARIES}
)

add_executable(rough_class
  src/rough_class.cpp
)
add_dependencies(rough_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rough_class
  ${catkin_LIBRARIES}
)