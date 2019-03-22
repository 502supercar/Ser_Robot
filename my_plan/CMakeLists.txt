cmake_minimum_required(VERSION 2.8.3)
project(my_plan)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_visual_tools
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
             pcl_ros
             pcl_conversions
             rosbag
             tf2_ros
             tf2_eigen
             tf2_geometry_msgs
             actionlib
             move_base_msgs
             roscpp
             tf
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

find_package(kinova_driver REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${kinova_driver_INCLUDE_DIRS})

add_executable(draw_retangle src/draw_retangle.cpp)
target_link_libraries(draw_retangle ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
#install(TARGETS draw_retangle DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(main src/main.cpp src/mobile_base.cpp src/my_pick_place.cpp include/mobile_base.h include/my_pick_place.h )
target_link_libraries(main ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
#install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
