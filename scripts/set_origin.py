#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg

import os

os.environ["MAVLINK20"] = "1"

VEHICLE_NAME = os.environ.get("VEHICLE_NAME", "duckiedrone")


def send_set_gps_global_origin():
    rospy.init_node("set_gps_global_origin")

    topic_name = f"/{VEHICLE_NAME}/mavlink/to"
    mavlink_pub = rospy.Publisher(topic_name, Mavlink, queue_size=10)

    rospy.loginfo(f"Publishing to topic: {topic_name}")

    rospy.sleep(1)  # Ensure the publisher is connected

    lat = 425633500  # Example latitude in degrees * 1E7
    lon = 126432900  # Example longitude in degrees * 1E7
    alt = 163000  # Altitude in mm

    # Create MAVLink message
    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
        target_system=1,  # Set to 1 (or 0 for broadcast)
        latitude=lat,
        longitude=lon,
        altitude=alt,
    )
    # **Explicitly Pack the Message**
    mav = MAV_APM.MAVLink(None, srcSystem=1)
    msg.pack(mav)  # Pack the message before converting

    # Convert MAVLink message to ROS format
    ros_msg = convert_to_rosmsg(msg)

    rospy.loginfo("Sending SET_GPS_GLOBAL_ORIGIN message to ")
    mavlink_pub.publish(ros_msg)
    rospy.sleep(1)


if __name__ == "__main__":
    try:
        send_set_gps_global_origin()
    except rospy.ROSInterruptException:
        pass
