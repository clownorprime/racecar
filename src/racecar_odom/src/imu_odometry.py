#! /usr/bin/env python

import rospy
import math
import tf2_ros
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class Odom:
    odom_topic = Odometry();
    odom_tf = TransformStamped();
    yaw_angle = 0
    old_yaw_angle = 0
    old_position = 0
    # the initial location of the robot
    start_x = -0.5
    start_y = 0
    start_z = 0
    def __init__(self):
        rospy.Subscriber("/imu_data", Imu, self.imu_callback, queue_size = 1);
        rospy.Subscriber("/racecar/joint_states", JointState, self.joint_states_callback, queue_size=1);
        self.odom_pub = rospy.Publisher("imu_odom", Odometry, queue_size = 1);
        self.tf = tf2_ros.TransformBroadcaster();
        self.rate = rospy.Rate(10);

        self.now_time = rospy.Time().to_sec();
        self.old_time = rospy.Time().to_sec();

        self.odom_topic.pose.pose.position.x = self.start_x;
        self.odom_topic.pose.pose.position.y = self.start_y;
        self.odom_topic.pose.pose.position.z = self.start_z;
        self.odom_topic.header.frame_id = "odom";
        self.odom_topic.child_frame_id = "base_footprint";

        self.odom_tf.header.frame_id = "odom";
        self.odom_tf.child_frame_id = "base_footprint";

    def imu_callback(self, data):
        self.yaw_angle = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w));
        self.odom_topic.pose.pose.orientation.x = data.orientation.x;
        self.odom_topic.pose.pose.orientation.y = data.orientation.y;
        self.odom_topic.pose.pose.orientation.z = data.orientation.z;
        self.odom_topic.pose.pose.orientation.w = data.orientation.w;

        self.odom_topic.twist.twist.angular.x = data.angular_velocity.x;
        self.odom_topic.twist.twist.angular.y = data.angular_velocity.y;
        self.odom_topic.twist.twist.angular.z = data.angular_velocity.z;

    def joint_states_callback(self, data):
        self.now_time = rospy.Time().to_sec();
        duration = self.now_time - self.old_time;
        self.old_time = self.now_time;

        self.odom_topic.header.stamp = rospy.Time().now();
        self.odom_tf.header.stamp = rospy.Time().now();

        # left and right rear speed
        lr_speed = data.velocity[data.name.index('left_rear_axle')];
        rr_speed = data.velocity[data.name.index('right_rear_axle')];

        # average speed 
        # 0.042 the radius of the back wheel
        speed = (lr_speed + rr_speed) / (2 * 13.95);

        position = speed * duration;

        self.odom_topic.pose.pose.position.x += (position * math.cos(self.yaw_angle[2]));
        self.odom_topic.pose.pose.position.y += (position * math.sin(self.yaw_angle[2]));

        self.odom_topic.pose.pose.position.z += 0;

        if ((self.yaw_angle[2] < 1.52) and (self.yaw_angle[2] > -1.52)):
            self.odom_topic.twist.twist.linear.x = speed * math.cos(self.yaw_angle[2]);
        else:
            self.odom_topic.twist.twist.linear.x = -speed * math.cos(self.yaw_angle[2]);

        self.odom_topic.twist.twist.linear.y = speed * math.sin(self.yaw_angle[2]);
        self.odom_topic.twist.twist.linear.z = 0;

        self.odom_pub.publish(self.odom_topic);

        self.odom_tf.transform.translation.x = self.odom_topic.pose.pose.position.x;
        self.odom_tf.transform.translation.y = self.odom_topic.pose.pose.position.y;
        self.odom_tf.transform.translation.z = self.odom_topic.pose.pose.position.z;

        self.odom_tf.transform.rotation.x = self.odom_topic.pose.pose.orientation.x;
        self.odom_tf.transform.rotation.y = self.odom_topic.pose.pose.orientation.y;
        self.odom_tf.transform.rotation.z = self.odom_topic.pose.pose.orientation.z;
        self.odom_tf.transform.rotation.w = self.odom_topic.pose.pose.orientation.w;

        self.tf.sendTransform(self.odom_tf);
def main():
    rospy.init_node("odom_imu")
    node = Odom();
    rospy.spin();

if __name__ == '__main__':
    main();

