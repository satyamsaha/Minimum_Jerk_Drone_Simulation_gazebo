import rospy
import csv
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, RCOut
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Initialize synchronized data lists
synchronized_data = []

def synchronized_callback(velocity_msg, pose_msg, rcout_msg):
    timestamp = rospy.Time.now().to_sec()

    velocity_data = [velocity_msg.twist.linear.x, velocity_msg.twist.linear.y, velocity_msg.twist.linear.z]
    angular_velocity_data = [velocity_msg.twist.angular.x, velocity_msg.twist.angular.y, velocity_msg.twist.angular.z]
    pose_orientation = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
    pose_data = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z] + pose_orientation
    pwm_values = list(rcout_msg.channels[:4])  # Convert tuple to list

    synchronized_data.append([timestamp] + velocity_data + angular_velocity_data + pose_data + pwm_values)

if __name__ == "__main__":
    rospy.init_node("data_recorder")

    velocity_sub = Subscriber("/mavros/local_position/velocity_local", TwistStamped)
    pose_sub = Subscriber("/mavros/local_position/pose", PoseStamped)
    rcout_sub = Subscriber("/mavros/rc/out", RCOut)

    ts = ApproximateTimeSynchronizer([velocity_sub, pose_sub, rcout_sub], queue_size=10, slop=0.1)
    ts.registerCallback(synchronized_callback)

    rate = rospy.Rate(100)  # Recording rate in Hz

    while not rospy.is_shutdown():
        rate.sleep()

        if len(synchronized_data) > 0:
            # Print some rows from synchronized_data for debugging
            print("Sample synchronized_data row:", synchronized_data[-1])

        # Get the current state of the drone
        current_state = None  # Initialize with None
        try:
            current_state = rospy.wait_for_message("/mavros/state", State, timeout=1.0)
        except rospy.ROSException:
            pass

        # Check if the drone is armed and guided to start recording
        recording_started = current_state and current_state.armed and current_state.guided

        if recording_started:
            rospy.loginfo("Recording started.")
            break  # Exit loop after recording starts

    rospy.sleep(1.0)  # Add a delay to ensure enough data is collected

    while not rospy.is_shutdown():
        rate.sleep()

        if recording_started:
            # Save synchronized data to CSV file
            file_name = "synchronized_data.csv"
            with open(file_name, "w") as csv_file:
                writer = csv.writer(csv_file)
                header_row = ["Time",
                              "Linear_Vel_X", "Linear_Vel_Y", "Linear_Vel_Z",
                              "Angular_Vel_X", "Angular_Vel_Y", "Angular_Vel_Z",
                              "Position_X", "Position_Y", "Position_Z",
                              "Orientation_X", "Orientation_Y", "Orientation_Z", "Orientation_W",
                              "PWM1", "PWM2", "PWM3", "PWM4"]
                writer.writerow(header_row)

                for row in synchronized_data:
                    writer.writerow(row)

            rospy.loginfo("Data saved to {}".format(file_name))
            break  # Exit loop after recording and saving data

    rospy.loginfo("Data collection complete.")
