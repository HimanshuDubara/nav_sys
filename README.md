# nav_sys

### File to launch all nodess - roslaunch navigation_pkg all_nodes.launch

For Localization (Code is in main branch)

Getting Turtlebot running: roslaunch turtlebot3_gazebo turtlebot3_world.launch

Getting UWB sensors running: roslaunch pozyx_simulation uwb_simulation_initializing.launch

Broadcast transform : rosrun learning_tf2 static_turtle_tf2_broadcaster.py odom 0 0 0 0 0 0

Getting Kalman Filters Running: roslaunch advoard_localization uwb_initial_pose.launch

Launching obstacle detector: roslaunch learning_tf2 detect.launch

To check the localization info: rostopic echo /localization_data_topic

Run Teleoperation node to moev the bot around to check localization values: roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

<node pkg="navigation_pkg" type="nav_algo_1.py" name="code" output="screen" launch-prefix="gnome-terminal --command" />

rosrun navigation_pkg nav_algo_1.py

<node name="detect1" pkg="learning_tf2" type="detect_all1.py" args="--test" respawn="true" />
<node name="detect2" pkg="learning_tf2" type="detect_all2.py" args="--test" respawn="true" />
<node name="detect3" pkg="learning_tf2" type="detect_all3.py" args="--test" respawn="true" />
<node name="detect4" pkg="learning_tf2" type="detect_all4.py" args="--test" respawn="true" />
<node name="detect5" pkg="learning_tf2" type="detect_all5.py" args="--test" respawn="true" />
<node name="detect6" pkg="learning_tf2" type="detect_all6.py" args="--test" respawn="true" />
<node name="detect7" pkg="learning_tf2" type="detect_all7.py" args="--test" respawn="true" />


Random
