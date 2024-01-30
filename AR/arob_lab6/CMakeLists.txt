cmake_minimum_required(VERSION 3.0.2)
project(arob_lab6)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg mav_trajectory_generation_ros mav_trajectory_generation_example)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)
include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(drone_race src/drone_race.cpp)
add_dependencies(drone_race ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_race ${catkin_LIBRARIES} Eigen3::Eigen)