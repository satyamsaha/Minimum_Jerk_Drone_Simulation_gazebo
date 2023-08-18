#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

path = Path()

def pose_callback(msg):
    global path

    # Add current pose to the path
    path.poses.append(msg)

    # Publish the path
    path_pub.publish(path)

if __name__ == "__main__":
    rospy.init_node("path_visualization_node")

    # Create a subscriber to receive the drone's pose
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)

    # Create a publisher to publish the path
    path_pub = rospy.Publisher("path", Path, queue_size=10)

    # Set up the path
    path.header.frame_id = "map"

    rospy.spin()
