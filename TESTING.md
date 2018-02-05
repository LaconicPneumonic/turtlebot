
# Testing

## Mapping

1. Open 6 tabs
	* 1 on the remote computer
	* 5 on the turtle bot
2. Run the following commands in the 5 turtle bot terminals
	1. ```> roslaunch turtlebot_bringup minimal.launch --screen```
	2. ```> rosrun hokuyo_node hokuyo_node```
	3. ```> rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint laser 100```
	4. ```> roslaunch turtlebot_navigation gmapping_demo.launch```
3. Run the following commands in the computer terminal
	1. ```> rqt_remocon```
	2. ```> roslaunch turtlebot_teleop keyboard_teleop.launch```
4. Choose Visualization
5. Choose View Robot
6. Add Laserscan if not already added
7. Add Map
8. Drive using the teleop terminal opened on the remote computer
9. Save map using the following
	1. ```> rosrun map_server map_saver -f MAP_LOCATION/my_map```

## Autonomous Driving
Follow the instructions [__HERE__] (http://learn.turtlebot.com/2015/02/01/12/) to test autonomous driving
