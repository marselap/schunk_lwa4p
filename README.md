# readme


## Build prerequisites

some ros packages... +

* [ros_controllers](https://github.com/otamachan/ros_controllers.git)
* [schunk_canopen_driver](github.com/matkok/schunk_canopen_driver.git)
    - config/joint_limits.yaml: `max_velocity: 0.5` and `max_acceleration: 1.1` (for consistency with HoCook calculation in schunk_lwa4p_trajectory pkg)
    - package.xml: remove `<run_depend>ros_controllers</run_depend>`
	- maybe add to urdf/lwa4p/lwa4p.gazebo.xacro (not sure if necessary)
		```
		<gazebo>
                 <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
		         </plugin>
       </gazebo>
		```
* [fzi_icl_can](https://github.com/fzi-forschungszentrum-informatik/fzi_icl_can)
* [fzi_icl_core](https://github.com/fzi-forschungszentrum-informatik/fzi_icl_core)
* [schunk_lwa4p](https://github.com/marselap/schunk_lwa4p)


## schunk_lwa4p

Files brief

### lwa4p_blue_control

*  	lwa4pBlueAdaptiveControlNode

    sub: /lwa4p_blue/operation_mode (if != 0 impedance control on)  
    sub: /lwa4p_blue/position_reference    
    sub: /lwa4p_blue/force_reference  
    sub: /lwa4p_blue/orientation_x  
    sub: /lwa4p_blue/orientation_z  
	sub: /lwa4p_blue/ft_sensor_topic (*simulator* force sensor readings)  
    sub: /lwa4p_blue/joint_states (*simulator* joint states)
    

### schunk_lwa4p_kinematics

* guiControlNode.cpp
    
    sub: /lwa4p_blue/reference_point (9x1 vector [position orient_x orient_z])  
    sub: /lwa4p_blue/joint_states (*simulator* joint states)  
    pub: /lwa4p_blue/waypoints (--> trajectoryPlanningNode --> listenTrajectory.py)  
    fcn: interpolates 3 points in joint space between current position and reference point as conditions for HoCook trajectory planning. Should be used before impedance control start to bring robot to initial position. 

### schunk_lwa4p_gazebo

* adaptive_wall.launch
    
    launch: lwa4p_blue, lwa4p_blue_controller, wall, empty_world
    nodes: spawn_robot, trajectoryPlanningNode, listenTrajectory.py, lwa4pBlueAdaptiveControlNode, forceAmplitudeNode
