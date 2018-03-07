cmake_minimum_required(VERSION 2.8.3)
project(schunk_lwa4p_kinematics)

find_package(Boost REQUIRED
          system
          filesystem
          date_time
          thread)

find_package(catkin REQUIRED COMPONENTS
          roscpp
          rospy
          roslib
          cmake_modules
          geometry_msgs
          sensor_msgs
          std_msgs
	  schunk_lwa4p_trajectory
)



catkin_package(
    INCLUDE_DIRS src include config
    LIBRARIES  ${Boost_LIBRARIES} lwa4p_kinematics yaml-cpp
    ## LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  geometry_msgs roscpp rospy std_msgs sensor_msgs
    DEPENDS system_lib boost
)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_library(lwa4p_kinematics src/lwa4p_kinematics.cpp)
add_dependencies(lwa4p_kinematics ${catkin_EXPORTED_TARGETS})
target_link_libraries(lwa4p_kinematics ${catkin_LIBRARIES})


add_library(lwa4p_kinematics_test src/lwa4p_kinematics_test.cpp)
target_link_libraries(lwa4p_kinematics_test ${catkin_LIBRARIES})

add_library(gui_control src/gui_control.cpp)
target_link_libraries(gui_control ${catkin_LIBRARIES})

add_library(current_position src/current_position.cpp)
target_link_libraries(current_position ${catkin_LIBRARIES})

add_executable(kinematicsTestNode nodes/kinematicsTestNode.cpp)
target_link_libraries(kinematicsTestNode  lwa4p_kinematics_test lwa4p_kinematics    ${catkin_LIBRARIES}  yaml-cpp)
add_dependencies(kinematicsTestNode lwa4p_kinematics ${catkin_EXPORTED_TARGETS})

add_executable(guiControlNode nodes/guiControlNode.cpp)
target_link_libraries(guiControlNode  gui_control lwa4p_kinematics    ${catkin_LIBRARIES}  yaml-cpp)
add_dependencies(guiControlNode lwa4p_kinematics ${catkin_EXPORTED_TARGETS})

add_executable(currentPositionNode nodes/currentPositionNode.cpp)
target_link_libraries(currentPositionNode  current_position lwa4p_kinematics    ${catkin_LIBRARIES}  yaml-cpp)
add_dependencies(currentPositionNode lwa4p_kinematics ${catkin_EXPORTED_TARGETS})
