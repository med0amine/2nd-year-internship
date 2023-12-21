This internship aims to implement the Enova Robotics test robot with autonomous navigation, programmed in Python, C/C++ and XML, using MakeFile and Cmake compilers in ROS Noetic and Ubuntu Linux. 
The system works in 3 main steps:

1. setting up the environment:
   
- Upload the robot model and the test field to Gazebo.
  
- Run the keyboard teleoperation node to control the robot in the Gazebo simulation.
  
2. slam simulation:
   
- Run the slam simulation with the gmapping package in RViz in parallel with Gazebo.
  
- Move the robot around so that it can detect every obstacle in the field with its lidar sensor.
  
- When the test field is displayed in RViz, download the maps into a YAML file.

3. start autonomous navigation

- Run the navigation package with the map

- Select the robot's destination in RViz so that the robot moves to that destination, taking the shortest possible path while avoiding obstacles.

This internship was a great opportunity to learn ROS and familiarise myself with its nodes, topics and services, as well as my first experience with XML files, MakeFile and CMake compilers.
