#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

input_list = [ 1, 'odom', 0, 0, 0, 0, 0, 0]

if __name__ == '__main__':
    if len(input_list) < 8:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './static_turtle_tf2_broadcaster.py '
                     'child_frame_name x y z roll pitch yaw')
        sys.exit(0)
    else:
        if input_list[1] == 'map':
            rospy.logerr('Your static turtle name cannot be "map"')
            sys.exit(0)

        rospy.init_node('my_static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = input_list[1]

        static_transformStamped.transform.translation.x = float(input_list[2])
        static_transformStamped.transform.translation.y = float(input_list[3])
        static_transformStamped.transform.translation.z = float(input_list[4])

        quat = tf.transformations.quaternion_from_euler(
                   float(input_list[5]),float(input_list[6]),float(input_list[7]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()
