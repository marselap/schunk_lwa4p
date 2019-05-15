cmake_minimum_required(VERSION 2.8.3)
project(schunk_lwa4p_trajectory)

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
          std_msgs
          moveit_msgs
          message_generation
)


add_message_files(
  FILES
  TrajectorySampled.msg
  WaypointArray.msg
  Waypoint.msg
)

##add_service_files(
##    FILES
##    SendHelixParams.srv
##    StandStill.srv
##    AssessPath3D.srv
##    GetLiftPoints.srv
##    PlanPath.srv
##    WaypointSrv.srv
##    SendPose.srv
##    WaypointArraySrv.srv
##    IsPointValid.srv
##    GetTakeoffAndLandPoints.srv
##    GetPath.srv
##   ReturnRMS.srv
##    )


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    ##actionlib_msgs
    geometry_msgs
    std_msgs
    moveit_msgs

)


catkin_package(
    INCLUDE_DIRS src include
    LIBRARIES  ${Boost_LIBRARIES} yaml-cpp
    CATKIN_DEPENDS  message_runtime geometry_msgs roscpp rospy std_msgs moveit_msgs
    
    DEPENDS system_lib boost
)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
## install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(hoCookTrajectory src/hoCookTrajectory.cpp)
add_dependencies(hoCookTrajectory schunk_lwa4p_trajectory_generate_messages_cpp)
target_link_libraries(hoCookTrajectory   ${catkin_LIBRARIES} )

add_executable(trajectoryPlanningNode nodes/trajectoryPlanningNode.cpp)
add_dependencies(hoCookTrajectory schunk_lwa4p_trajectory_generate_messages_cpp)
target_link_libraries(trajectoryPlanningNode hoCookTrajectory   ${catkin_LIBRARIES} yaml-cpp )
