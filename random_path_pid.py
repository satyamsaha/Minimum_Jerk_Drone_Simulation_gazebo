#! /usr/bin/env python3

import rospy
import math
import numpy as np
import random
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import Quaternion

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    quaternion = Quaternion()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    duration = rospy.Duration(100)  # Total duration of circular flight (60 seconds)
    start_time = rospy.Time.now()
   
    xi = [0, 0, 0]
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()
        
        end_time = start_time + duration
        
        if rospy.Time.now() < end_time:
            # Calculate desired position in the circular trajectory
            t = 0
            dt = .001
            #d = .5
            d = 25
            while t < d:
                xf = [random.uniform(-10, 10), random.uniform(-12, 12), random.uniform(0, 5)]
                x=np.zeros((3,1))
            # Calculate the desired position using polynomial interpolation
                #x=np.array(xi) + (np.array(xf) - np.array(xi)) * (10 * (t / d) ** 3 - 15 * (t / d) ** 4 + 6 * (t / d) ** 5)
                #s2D + (g2D - s2D)*(10*(t[i]/t_mission)**3  - 15*(t[i]/t_mission)**4 + 6*(t[i]/t_mission)**5)
                x=np.array(xi) + (np.array(xf) - np.array(xi)) * (10 * (t / d) ** 3 - 15 * (t / d) ** 4 + 6 * (t / d) ** 5)
                t += dt
                pose.pose.position.x = x[0]
                pose.pose.position.y = x[1]
                pose.pose.position.z = x[2]

            # Debug statements
            #rospy.loginfo("Desired Position: x={}, y={}, z={}".format(x[0], x[1], x[2]))
            #rospy.loginfo("Current Position: x={}, y={}, z={}".format(
                #current_pose.pose.position.x,
                #current_pose.pose.position.y,
                #current_pose.pose.position.z
            #))

            local_pos_pub.publish(pose)
            #xi = x
        
        else:
            # Initiate landing mode
            land_set_mode = SetModeRequest()
            land_set_mode.custom_mode = "AUTO.LAND"

            if current_state.mode != "AUTO.LAND":
                set_mode_client(land_set_mode)
                rospy.loginfo("Landing mode enabled")

        rate.sleep()
