cmake_minimum_required(VERSION 2.8.3)
project(lwa4p_blue_control)

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
	  schunk_lwa4p_kinematics
	  message_generation
)




catkin_package(
    INCLUDE_DIRS src include
    LIBRARIES  ${Boost_LIBRARIES} yaml-cpp
    CATKIN_DEPENDS  message_runtime geometry_msgs roscpp rospy std_msgs
    DEPENDS system_lib boost
)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(pid_controller_base src/pid_controller_base.cpp)
target_link_libraries(pid_controller_base  ${catkin_LIBRARIES})

add_library(adaptive_pd src/adaptive_pd.cpp)
target_link_libraries(adaptive_pd  ${catkin_LIBRARIES})

add_library(lwa4pBlueControl src/lwa4pBlueControl.cpp)
target_link_libraries(lwa4pBlueControl  ${catkin_LIBRARIES})

add_library(forceAmplitude src/forceAmplitude.cpp)
target_link_libraries(forceAmplitude  ${catkin_LIBRARIES})

add_library(adaptiveImpedanceControl src/adaptiveImpedanceControl.cpp)
target_link_libraries(adaptiveImpedanceControl  ${catkin_LIBRARIES})

add_executable(lwa4pBlueControlNode nodes/lwa4pBlueControlNode.cpp)
target_link_libraries(lwa4pBlueControlNode  lwa4pBlueControl pid_controller_base pid_controller_base ${catkin_LIBRARIES}  yaml-cpp)

add_executable(forceAmplitudeNode nodes/forceAmplitudeNode.cpp)
target_link_libraries(forceAmplitudeNode  forceAmplitude ${catkin_LIBRARIES}  yaml-cpp)

add_executable(lwa4pBlueAdaptiveControlNode nodes/lwa4pBlueAdaptiveControlNode.cpp)
target_link_libraries(lwa4pBlueAdaptiveControlNode  adaptiveImpedanceControl adaptive_pd ${catkin_LIBRARIES}  yaml-cpp)
