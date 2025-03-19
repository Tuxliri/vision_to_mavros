import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import math

#!/usr/bin/env python


def publish_circular_trajectory(vehicle_name):
    rospy.init_node('circular_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher(f"/{vehicle_name}/duckiematrix_interface_node/state", Odometry, queue_size=10)
    rate = rospy.Rate(50)  # 10 Hz

    radius = 10.0  # radius of the circle
    angular_velocity = 0.3  # angular velocity in radians per second
    current_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        time_elapsed = (current_time - rospy.Time(0)).to_sec()

        x = radius * math.cos(angular_velocity * time_elapsed)
        y = radius * math.sin(angular_velocity * time_elapsed)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
        odom.twist.twist = Twist()

        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        vehicle_name = "endeavour"  # Replace with your vehicle name
        publish_circular_trajectory(vehicle_name)
    except rospy.ROSInterruptException:
        pass