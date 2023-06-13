## Loading Gazebo, Rviz and python script


## Feng
export TURTLEBOT_GAZEBO_WORLD_FILE=$HOME/catkin_ws/src/group_project/project/launch/project.world \
vglrun roslaunch turtlebot_gazebo turtlebot_world.launch \
cd ./catkin_ws/src/group_project/project/launch \
vglrun roslaunch simulated_localisation.launch map_file:=$HOME/catkin_ws/src/group_project/project/example/map/project_map.yaml \
vglrun roslaunch turtlebot_rviz_launchers view_navigation.launch \
rosrun project Cluedo_Character_Finder.py


## Native
export TURTLEBOT_GAZEBO_WORLD_FILE=$HOME/catkin_ws/src/group_project/project/launch/project.world \
roslaunch turtlebot_gazebo turtlebot_world.launch \
cd ./catkin_ws/src/group_project/project/launch \
roslaunch $HOME/catkin_ws/src/group_project/project/launch/simulated_localisation.launch map_file:=$HOME/catkin_ws/src/group_project/project/example/map/project_map.yaml \
roslaunch turtlebot_rviz_launchers view_navigation.launch \
roslaunch $HOME/catkin_ws/src/group_project/project/launch/cludeo_character_finder.launch 
