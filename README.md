# nav_sys
For Localization (Code is in main branch)

Getting Turtlebot running: roslaunch turtlebot3_gazebo turtlebot3_world.launch

Getting UWB sensors running: roslaunch pozyx_simulation uwb_simulation_initializing.launch

Broadcast transform : rosrun learning_tf2 static_turtle_tf2_broadcaster.py odom 0 0 0 0 0 0

Getting Kalman Filters Running: roslaunch advoard_localization uwb_initial_pose.launch

To check the localization info: rostopic echo /localization_data_topic

Run Teleoperation node to moev the bot around to check localization values: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Random
