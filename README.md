# ROS-AStar
Package contains test world with built map, launch files to start mapping, launch amcl or A*.
To create new map:

    roslaunch goto_mover mapping.launch 
To launch A* planner separately on the map run:

    rosrun goto_mover planner.py
To launch program to control standard amcl path planner run:

    roslaunch goto_mover goto_amcl.launch   
To launch program to control A* path planner run:

    roslaunch goto_mover goto_astar.launch
