cmake_minimum_required(VERSION 3.0.2)

project(time_pub_sub_pkg)  #<--- TODO: change this to your own package name
 
## Compile as C++11, supported in ROS Noetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS  #<--- TODO: Add your required packages here
  roscpp
  rospy
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs  #TODO: Add your required packages here
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
)


add_executable(example_time_publisher src/example_time_publisher.cpp) #TODO: change the name of the executable to your own executable name

target_link_libraries(example_time_publisher ${catkin_LIBRARIES}) #TODO: change the name of the executable to your own executable name


install(PROGRAMS
  scripts/example_time_subscriber.py #TODO: change the name of the executable to your own executable name
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS example_time_publisher #TODO: change the name of the executable to your own executable name
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)