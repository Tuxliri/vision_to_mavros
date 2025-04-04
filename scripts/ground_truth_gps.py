#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped

VEHICLE_NAME = 'endeavour'

def odom_callback(odom_msg):
    # Publish pose
    pose_msg = PoseStamped()
    pose_msg.header = odom_msg.header
    pose_msg.pose = odom_msg.pose.pose
    pose_pub.publish(pose_msg)

    # Publish twist
    twist_msg = TwistWithCovarianceStamped()
    twist_msg.header = odom_msg.header
    twist_msg.twist.twist = odom_msg.twist.twist
    twist_pub.publish(twist_msg)
    rospy.loginfo("Published pose and twist")

if __name__ == '__main__':
    rospy.init_node('odom_to_pose_and_twist_converter')
    pose_pub = rospy.Publisher(f"/{VEHICLE_NAME}/mavros/vision_pose/pose", PoseStamped, queue_size=10)
    twist_pub = rospy.Publisher(f"/{VEHICLE_NAME}/mavros/vision_speed/speed_twist_cov", TwistWithCovarianceStamped, queue_size=10)
    rospy.Subscriber(f"/{VEHICLE_NAME}/duckiematrix_interface_node/state", Odometry, odom_callback)
    rospy.spin()
