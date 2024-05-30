#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdometryError:
    def __init__(self):
        self.gazebo_odom = rospy.Subscriber("gazebo_odom", Odometry, self.gazebo_odom_callback);
        self.imu_odom = rospy.Subscriber("imu_odom", Odometry, self.imu_odom_callback);
        self.error_pub = rospy.Publisher("odom_error", Odometry, queue_size=1);
    def gazebo_odom_callback(self, data):
        self.real_odom = data;
        RPY = euler_from_quaternion(
                (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w));
        self.real_odom.pose.pose.orientation.w = RPY[2];
    def imu_odom_callback(self, data):
        error = Odometry();

        error.header.stamp = rospy.Time().now();
        error.header.frame_id = "gazebo_odom";
        error.child_frame_id = "imu_odom";

        error.pose.pose.position.x = self.real_odom.pose.pose.position.x - data.pose.pose.position.x;
        error.pose.pose.position.y = self.real_odom.pose.pose.position.y - data.pose.pose.position.y;
        error.pose.pose.position.z = self.real_odom.pose.pose.position.z - data.pose.pose.position.z;
        RPY = euler_from_quaternion(
                (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w));
        error.pose.pose.orientation.w = RPY[2] - self.real_odom.pose.pose.orientation.w;
        self.error_pub.publish(error);
def main():
    rospy.init_node("error_odom")
    node = OdometryError();
    rospy.spin();

if __name__ == '__main__':
    try:
        main();
    except rospy.ROSInterruptException:
        pass
