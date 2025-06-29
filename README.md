# multi-robot-navigation

## Project Code Structure:
├── d3qn_navigation - Reinforcement learning navigation node directory  
│   ├── CMakeLists.txt - Build file  
│   ├── include - Header files directory  
│   │   └── d3qn_navigation - Header files directory  
│   ├── launch - ROS launch files
│   │   └── d3qn_navigation_main.launch - Launch file for reinforcement learning navigation node
│   ├── package.xml - ROS package data file
│   ├── scripts - Python scripts directory
│   │   ├── behavior_choice_d3qn.py - Behavior choice model D3QN algorithm code
│   │   ├── behavior_choice_d3qn.pyc - Compiled version of behavior_choice_d3qn.py
│   │   ├── behavior_choice_env.py - Behavior choice model D3QN environment code
│   │   ├── behavior_choice_env.pyc - Compiled version of behavior_choice_env.py
│   │   ├── buffer.py - Experience replay code
│   │   ├── buffer.pyc - Compiled version of buffer.py
│   │   ├── d3qn_navigation_main.py - Main function code for reinforcement learning navigation
│   │   ├── listeners.py - Listener code for reinforcement learning navigation node
│   │   ├── listeners.pyc - Compiled version of listeners.py
│   │   ├── navigation_control_d3qn.py - Obstacle avoidance control model D3QN algorithm code
│   │   ├── navigation_control_d3qn.pyc - Compiled version of navigation_control_d3qn.py
│   │   ├── navigation_control_env.py - Obstacle avoidance control model D3QN environment code
│   │   └── navigation_control_env.pyc - Compiled version of navigation_control_env.py
│   └── src - C++ scripts directory (not used)
├── def_msgs - Custom message package node files
│   ├── CMakeLists.txt - Build file
│   ├── include
│   │   └── def_msgs
│   ├── msg
│   │   ├── behavior.msg - Custom message type for behavior publishing
│   │   ├── follower_yaw.msg - Custom message type for follower yaw
│   │   └── pose.msg - Custom message type for robot pose
│   ├── package.xml
│   └── src
├── leader_follow - Leader-Follower node directory
│   ├── CMakeLists.txt
│   ├── include
│   │   └── leader_follow
│   ├── launch
│   │   └── leader_follow_main.launch - Launch file for leader-follower node
│   ├── package.xml
│   ├── scripts - Python scripts directory
│   │   ├── leader_follow_main.py - Leader-follower local control code
│   │   └── robot_poses_publisher.py - Code for the robot poses publisher node
│   └── src
├── multi_robot_amcl - ROS multi-robot AMCL localization node
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── amcl_robot1.launch - Launch file for multi-robot AMCL algorithm (robot 1)
│   │   ├── amcl_robot2.launch - Launch file for multi-robot AMCL algorithm (robot 2)
│   │   ├── amcl_robot3.launch - Launch file for multi-robot AMCL algorithm (robot 3)
│   │   ├── move_base_1.launch
│   │   ├── move_base_2.launch 
│   │   ├── move_base_3.launch
│   │   └── navigation.launch - Main launch file for the navigation node
│   └── package.xml
├── multi_robot_map_launcher - Multi-robot ROS map storage node
│   ├── CMakeLists.txt
│   ├── include
│   │   └── map_launcher
│   ├── map
│   │   ├── map.pgm - Map model
│   │   └── map.yaml - Map model
│   ├── package.xml
│   ├── rviz
│   │   ├── navigation.rviz
│   │   └── turtlebot3_navigation.rviz
│   └── src
├── multi_robot_sim - Multi-robot simulation node
│   ├── CMakeLists.txt
│   ├── include
│   │   └── multi_robot_sim
│   ├── launch
│   │   ├── main.launch - Main launch file for this project
│   │   ├── one_robot.launch - Launch file to place one robot
│   │   └── robots.launch - Launch file to place multiple robots
│   ├── package.xml
│   ├── sh
│   │   └── kill_sim.sh - Command code to terminate the process
│   ├── src
│   ├── urdf
│   │   ├── turtlebot3_burger.gazebo.xacro - Robot model file
│   │   └── turtlebot3_burger.urdf.xacro - Robot model file
│   └── worlds
│       └── corridor.world - World environment model file
└── plotter - Plotting node
    ├── CMakeLists.txt
    ├── include
    │   └── plotter
    ├── package.xml
    └── scripts
          └── leader_follow_error_plotter.py - Plotting code 
